#!/bin/python3

import rospy
import rostopic
from gym.envs.registration import register

import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ros_rl.envs import RealBaseEnv

# core modules of the framework
from ros_rl.utils.moveit_ros_rl import MoveitROS_RL
from ros_rl.utils import ros_common
from ros_rl.utils import ros_controllers
from ros_rl.utils import ros_markers

from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from tf.transformations import euler_from_matrix

"""
Although it is best to register only the task environment, one can also register the robot environment. 
This is not necessary, but we can see if this section works by calling "gym.make" this env.
"""
register(
    id='RX200RobotBaseEnv-v1',
    entry_point='reactorx200_ros_reacher.real.robot_envs.reactorx200_robot_real_v1:RX200RobotEnv',
    max_episode_steps=100,
)


class RX200RobotEnv(RealBaseEnv.RealBaseEnv):
    """
    RX200 Robot Env, use this class to describe the robots and the sensors in the Environment.
    Superclass for all Robot environments.

    This is the v1 of the robot environment. Following are the changes from the v0:
        * Use moveit check if the goal is reachable - joint positions
        * Get joint states for velocity and position
        * use ros_controllers to control the robot - more low-level control
        * FK loop to get the end-effector pose

    """

    def __init__(self, ros_port: str = None, seed: int = None, close_env_prompt: bool = False, action_cycle_time=0.0,
                 async_moveit=False):
        """
        Initializes a new Robot Environment

        Describe the robot and the sensors used in the env.

        Sensor Topic List:
            MoveIt: To get the pose and rpy of the robot.
            /joint_states: JointState received for the joints of the robot

        Actuators Topic List:
            MoveIt!: Send the joint positions to the robot.
            ROS Controllers: ROS Controllers are used to send the joint positions to the robot.
        """
        rospy.loginfo("Start Init RX200RobotEnv ROS_RL")

        """
        Change the ros/gazebo master
        """
        if ros_port is not None:
            ros_common.change_ros_master(ros_port=ros_port)

        """
        parameters
        """
        self.async_moveit = async_moveit

        """
        Launch a roslaunch file that will setup the connection with the real robot 
        """

        load_robot = True
        robot_pkg_name = "interbotix_xsarm_moveit_interface"
        robot_launch_file = "xsarm_moveit_interface.launch"
        robot_args = ["robot_model:=rx200", "use_actual:=true", "dof:=5", "use_python_interface:=true",
                      "use_moveit_rviz:=true"]

        """
        namespace of the robot
        """
        namespace = "/rx200"

        """
        kill rosmaster at the end of the env
        """
        kill_rosmaster = False

        """
        Clean ros Logs at the end of the env
        """
        clean_logs = True

        """
        Init GazeboBaseEnv.
        """
        super().__init__(
            load_robot=load_robot, robot_pkg_name=robot_pkg_name, robot_launch_file=robot_launch_file,
            robot_args=robot_args, namespace=namespace, kill_rosmaster=kill_rosmaster, clean_logs=clean_logs,
            ros_port=ros_port, seed=seed, close_env_prompt=close_env_prompt, action_cycle_time=action_cycle_time)

        """
        Using the _check_connection_and_readiness method to check for the connection status of subscribers, publishers 
        and services
        """
        self._check_connection_and_readiness()

        """
        initialise controller and sensor objects here
        """
        # ---------- joint state
        if namespace is not None and namespace != '/':
            self.joint_state_topic = namespace + "/joint_states"
        else:
            self.joint_state_topic = "/joint_states"

        self.joint_state_sub = rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_callback)
        self.joint_state = JointState()

        # Moveit object
        self.move_RX200_object = MoveitROS_RL(arm_name='interbotix_arm',
                                              gripper_name='interbotix_gripper',
                                              robot_description="rx200/robot_description",
                                              ns="rx200")

        # For ROS Controllers
        self.joint_names = ["waist",
                            "shoulder",
                            "elbow",
                            "wrist_angle",
                            "wrist_rotate"]

        # low-level control
        # The rostopic for joint trajectory controller
        self.joint_trajectory_controller_pub = rospy.Publisher('/rx200/arm_controller/command',
                                                               JointTrajectory,
                                                               queue_size=10)

        # parameters for calculating FK
        self.ee_link = "rx200/ee_gripper_link"
        self.ref_frame = "rx200/base_link"

        # Fk with pykdl_utils
        self.pykdl_robot = URDF.from_parameter_server(key='rx200/robot_description')
        self.kdl_kin = KDLKinematics(urdf=self.pykdl_robot, base_link=self.ref_frame, end_link=self.ee_link)

        """
        Finished __init__ method
        """
        rospy.loginfo("End Init RX200RobotEnv")

    # ---------------------------------------------------
    #   Custom methods for the Robot Environment

    """
    Define the custom methods for the environment
        * fk_pykdl: Function to calculate the forward kinematics of the robot arm. We are using pykdl_utils. 
        * move_joints: Set a joint position target only for the arm joints using low-level ros controllers.
        * joint_state_callback: Get the joint state of the robot
        * set_trajectory_joints: Set a joint position target only for the arm joints.
        * set_trajectory_ee: Set a pose target for the end effector of the robot arm.
        * get_ee_pose: Get end-effector pose a geometry_msgs/PoseStamped message
        * get_ee_rpy: Get end-effector orientation as a list of roll, pitch, and yaw angles.
        * get_joint_angles: Get current joint angles of the robot arm - 5 elements
        * check_goal: Check if the goal is reachable
        * check_goal_reachable_joint_pos: Check if the goal is reachable with joint positions
    """
    def fk_pykdl(self, action):
        """
        Function to calculate the forward kinematics of the robot arm. We are using pykdl_utils.

        Args:
            action: joint positions of the robot arm (in radians)

        Returns:
            ee_position: end-effector position as a numpy array
        """
        # Calculate forward kinematics
        pose = self.kdl_kin.forward(action)

        # Extract position
        ee_position = np.array([pose[0, 3], pose[1, 3], pose[2, 3]], dtype=np.float32)  # we need to convert to float32

        # Extract rotation matrix and convert to euler angles
        # ee_orientation = euler_from_matrix(pose[:3, :3], 'sxyz')

        return ee_position

    def joint_state_callback(self, joint_state):
        """
        Function to get the joint state of the robot.
        """

        self.joint_state = joint_state

        # joint names - not using this
        self.joint_state_names = list(joint_state.name)

        # get the current joint positions - using this
        joint_pos_all = list(joint_state.position)
        self.joint_pos_all = joint_pos_all

        # get the current joint velocities - we are using this
        self.current_joint_velocities = list(joint_state.velocity)

        # get the current joint efforts - not using this
        self.current_joint_efforts = list(joint_state.effort)

    def move_joints(self, q_positions: np.ndarray, time_from_start: float = 0.5) -> bool:
        """
        Set a joint position target only for the arm joints using low-level ros controllers.

        Args:
            q_positions: joint positions
            time_from_start: time from start for the trajectory (set the speed to complete the trajectory within this time)

        Returns:
            True if the trajectory is set
        """

        # create a JointTrajectory object
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = q_positions
        trajectory.points[0].velocities = [0.0] * len(self.joint_names)
        trajectory.points[0].accelerations = [0.0] * len(self.joint_names)
        trajectory.points[0].time_from_start = rospy.Duration(time_from_start)

        # send the trajectory to the controller
        self.joint_trajectory_controller_pub.publish(trajectory)

        return True

    def set_trajectory_joints(self, q_positions: np.ndarray) -> bool:
        """
        Set a joint position target only for the arm joints.
        """
        if self.async_moveit:
            return self.move_RX200_object.set_trajectory_joints(q_positions, async_move=True)
        else:
            return self.move_RX200_object.set_trajectory_joints(q_positions)

    def set_trajectory_ee(self, pos: np.ndarray) -> bool:
        """
        Set a pose target for the end effector of the robot arm.
        """
        if self.async_moveit:
            return self.move_RX200_object.set_trajectory_ee(position=pos, async_move=True)
        else:
            return self.move_RX200_object.set_trajectory_ee(position=pos)

    def get_ee_pose(self):
        """
        Returns the end-effector pose as a geometry_msgs/PoseStamped message

        This gives us the best pose if we are using the moveit config of the ReactorX repo
        They are getting the pose with ee_gripper_link
        """
        return self.move_RX200_object.get_robot_pose()

    def get_ee_rpy(self):
        """
        Returns the end-effector orientation as a list of roll, pitch, and yaw angles.
        """
        return self.move_RX200_object.get_robot_rpy()

    def get_joint_angles(self):
        """
        get current joint angles of the robot arm - 5 elements
        Returns a list
        """
        return self.move_RX200_object.get_joint_angles_robot_arm()

    def check_goal(self, goal):
        """
        Check if the goal is reachable
        """
        return self.move_RX200_object.check_goal(goal)

    def check_goal_reachable_joint_pos(self, joint_pos):
        """
        Check if the goal is reachable with joint positions
        """
        return self.move_RX200_object.check_goal_joint_pos(joint_pos)

    # helper fn for _check_connection_and_readiness
    def _check_joint_states_ready(self):
        """
        Function to check if the joint states are received
        """
        # Wait for the service to be available
        rospy.logdebug(rostopic.get_topic_type(self.joint_state_topic, blocking=True))

        return True

    def _check_moveit_ready(self):
        """
        Function to check if moveit services are running
        """
        rospy.wait_for_service("/rx200/move_group/trajectory_execution/set_parameters")
        rospy.logdebug(rostopic.get_topic_type("/rx200/planning_scene", blocking=True))
        rospy.logdebug(rostopic.get_topic_type("/rx200/move_group/status", blocking=True))

        return True

    # helper fn for _check_connection_and_readiness
    def _check_ros_controllers_ready(self):
        """
        Function to check if ros controllers are running
        """
        rospy.logdebug(rostopic.get_topic_type("/rx200/arm_controller/state", blocking=True))
        rospy.logdebug(rostopic.get_topic_type("/rx200/gripper_controller/state", blocking=True))

        return True

    # ---------------------------------------------------
    #   Methods to override in the Robot Environment

    def _check_connection_and_readiness(self):
        """
        Function to check the connection status of subscribers, publishers and services, as well as the readiness of
        all systems.
        """
        self._check_moveit_ready()
        self._check_joint_states_ready()
        self._check_ros_controllers_ready()

        rospy.loginfo("All system are ready!")
        return True

    # ---------------------------------------------------
    #    Methods to override in the Task Environment

    def _set_action(self, action):
        """
        Function to apply an action to the robot.

        This method should be implemented by subclasses to apply the given action to the robot. The action could be a
        joint position command, a velocity command, or any other type of command that can be applied to the robot.

        Args:
            action: The action to be applied to the robot.
        """
        raise NotImplementedError()

    def _get_observation(self):
        """
        Get an observation from the environment.

        This method should be implemented by subclasses to return an observation representing the current state of
        the environment. The observation could be a sensor reading, a joint state, or any other type of observation
        that can be obtained from the environment.

        Returns:
            observation (Any): An observation representing the current state of the environment.
        """
        raise NotImplementedError()

    def _get_reward(self):
        """
        Function to get a reward from the environment.

        This method should be implemented by subclasses to return a scalar reward value representing how well the agent
        is doing in the current episode. The reward could be based on the distance to a goal, the amount of time taken
        to reach a goal, or any other metric that can be used to measure how well the agent is doing.

        Returns:
            A scalar reward value representing how well the agent is doing in the current episode.
        """
        raise NotImplementedError()

    def _is_done(self):
        """
        Function to check if the episode is done.

        This method should be implemented by subclasses to return a boolean value indicating whether the episode has
        ended (e.g., because a goal has been reached or a failure condition has been triggered).

        Returns:
            A boolean value indicating whether the episode has ended
            (e.g., because a goal has been reached or a failure condition has been triggered)
        """
        raise NotImplementedError()

    def _set_init_params(self):
        """
        Set initial parameters for the environment.

        This method should be implemented by subclasses to set any initial parameters or state variables for the
        environment. This could include resetting joint positions, resetting sensor readings, or any other initial
        setup that needs to be performed at the start of each episode.
        """
        raise NotImplementedError()
