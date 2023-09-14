#!/bin/python3

from gym import spaces
from gym.envs.registration import register
import numpy as np

from multiros.envs import GazeboBaseEnv

import rospy
import rostopic
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

# core modules of the framework
from multiros.utils import gazebo_core
from multiros.utils import gazebo_models
from multiros.utils import gazebo_physics
from multiros.utils.moveit_multiros import MoveitMultiros
from multiros.utils import ros_common
from multiros.utils import ros_controllers
from multiros.utils import ros_markers

"""
Although it is best to register only the task environment, one can also register the robot environment. 
This is not necessary, but we can see if this section 
(Load the robot to gazebo and can control the robot with moveit or ros controllers)
works by calling "gym.make" this env.
but you need to
    1. run gazebo - gazebo_core.launch_gazebo(launch_roscore=False, paused=False, pub_clock_frequency=100, gui=True)
    2. init a node - rospy.init_node('test_MyRobotGoalEnv')
    3. gym.make("RX200RobotEnv-v0")
"""
register(
    id='RX200RobotEnv-v2',
    entry_point='reactorx200_ros_reacher.sim.robot_envs.reactorx200_robot_sim_v1:RX200RobotEnv',
    max_episode_steps=1000,
)


class RX200RobotEnv(GazeboBaseEnv.GazeboBaseEnv):
    """
    Superclass for all RX200 Robot environments.

    This the v2 of the robot environment. Following are the changes from the v1:
        * Added the option to run the simulation in real time - line 75 to 81 also pass it to the BaseEnv
        * Added the option to set the action cycle time - we can now pass it to the BaseEnv
        * Updated the moveit set trajectory functions to be able to run in real time
    """

    def __init__(self, ros_port: str = None, gazebo_port: str = None, gazebo_pid=None, seed: int = None,
                 real_time: bool = False, action_cycle_time=0.0):
        """
        Initializes a new Robot Environment

        Describe the robot and the sensors used in the env.

        Sensor Topic List:
            MoveIt: To get the pose and rpy of the robot.
            /joint_states: JointState received for the joints of the robot

        Actuators Topic List:
            MoveIt: Send the joint positions to the robot.
        """
        rospy.loginfo("Start Init RX200RobotEnv Multiros")

        """
        Change the ros/gazebo master
        """
        if ros_port is not None:
            ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

        """
        parameters
        """
        self.real_time = real_time  # if True, the simulation will run in real time

        # we don't need to pause/unpause gazebo if we are running in real time
        if self.real_time:
            unpause_pause_physics = False
        else:
            unpause_pause_physics = True

        """
        Unpause Gazebo
        """
        if not self.real_time:
            gazebo_core.unpause_gazebo()

        """
        Spawning the robot in Gazebo
        """
        spawn_robot = True

        # location of the robot URDF file
        urdf_pkg_name = "reactorx200_description"
        urdf_file_name = "rx200_kinect.urdf.xacro"
        urdf_folder = "/urdf"

        # extra urdf args
        urdf_xacro_args = None

        # namespace of the robot
        namespace = "/rx200"

        # robot state publisher
        robot_state_publisher_max_freq = None
        new_robot_state_term = False

        robot_model_name = "rx200"
        robot_ref_frame = "world"

        # Set the initial pose of the robot model
        robot_pos_x = 0.0
        robot_pos_y = 0.0
        robot_pos_z = 0.0
        robot_ori_w = 0.0
        robot_ori_x = 0.0
        robot_ori_y = 0.0
        robot_ori_z = 0.0

        # controller (must be inside above pkg_name/config/)
        controllers_file = "reactorx200_controller.yaml"
        controllers_list = ["joint_state_controller", "arm_controller", "gripper_controller"]

        """
        Spawn other objects in Gazebo
        """
        # uncomment and change the parameters
        # gazebo_models.spawn_sdf_model_gazebo(pkg_name=pkg_name, file_name="model.sdf", model_folder="/models/table",
        #                                      model_name="table", namespace=namespace, pos_x=0.2)

        """
        Set if the controllers in "controller_list" will be reset at the beginning of each episode, default is False.
        """
        reset_controllers = False

        """
        Set the reset mode of gazebo at the beginning of each episode
            "simulation": Reset gazebo simulation (Resets time) 
            "world": Reset Gazebo world (Does not reset time) - default

        resetting the "simulation" restarts the entire Gazebo environment, including all models and their positions, 
        while resetting the "world" retains the models but resets their properties and states within the world        
        """
        reset_mode = "world"

        """
        You can adjust the simulation step mode of Gazebo with two options:

            1. Using Unpause, set action and Pause gazebo
            2. Using the step function of Gazebo.

        By default, the simulation step mode is set to 1 (gazebo pause and unpause services). 
        However, if you choose simulation step mode 2, you can specify the number of steps Gazebo should take in each 
        iteration. The default value for this is 1.
        """
        sim_step_mode = 1
        num_gazebo_steps = 1

        """
        Set gazebo physics parameters to change the speed of the simulation
        """
        gazebo_max_update_rate = None
        gazebo_timestep = None

        if rospy.has_param('/rx200/update_rate_multiplier'):
            gazebo_max_update_rate = rospy.get_param('/rx200/gazebo_update_rate_multiplier')

        if rospy.has_param('/rx200/time_step'):
            gazebo_timestep = rospy.get_param('/rx200/time_step')

        """
        kill rosmaster at the end of the env
        """
        kill_rosmaster = True

        """
        kill gazebo at the end of the env
        """
        kill_gazebo = True

        """
        Clean ros Logs at the end of the env
        """
        clean_logs = False

        """
        Init GazeboBaseEnv.
        """
        super().__init__(
            spawn_robot=spawn_robot, urdf_pkg_name=urdf_pkg_name, urdf_file_name=urdf_file_name,
            urdf_folder=urdf_folder, urdf_xacro_args=urdf_xacro_args, namespace=namespace,
            robot_state_publisher_max_freq=robot_state_publisher_max_freq, new_robot_state_term=new_robot_state_term,
            robot_model_name=robot_model_name, robot_ref_frame=robot_ref_frame,
            robot_pos_x=robot_pos_x, robot_pos_y=robot_pos_y, robot_pos_z=robot_pos_z, robot_ori_w=robot_ori_w,
            robot_ori_x=robot_ori_x, robot_ori_y=robot_ori_y, robot_ori_z=robot_ori_z,
            controllers_file=controllers_file, controllers_list=controllers_list,
            reset_controllers=reset_controllers, reset_mode=reset_mode, sim_step_mode=sim_step_mode,
            num_gazebo_steps=num_gazebo_steps, gazebo_max_update_rate=gazebo_max_update_rate,
            gazebo_timestep=gazebo_timestep, kill_rosmaster=kill_rosmaster, kill_gazebo=kill_gazebo,
            clean_logs=clean_logs, ros_port=ros_port, gazebo_port=gazebo_port, gazebo_pid=gazebo_pid, seed=seed,
            unpause_pause_physics=unpause_pause_physics, action_cycle_time=action_cycle_time)

        """
        Define ros publisher, subscribers and services for robot and sensors
        """
        # ---------- joint state
        if namespace is not None and namespace != '/':
            self.joint_state_topic = namespace + "/joint_states"
        else:
            self.joint_state_topic = "/joint_states"

        self.joint_state_sub = rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_callback)
        self.joint_state = JointState()


        # ---------- Moveit
        ros_common.ros_launch_launcher(pkg_name="interbotix_xsarm_moveit_interface",
                                       launch_file_name="xsarm_moveit_interface.launch",
                                       args=["robot_model:=rx200", "dof:=5", "use_python_interface:=true",
                                             "use_moveit_rviz:=false"])

        """
        Using the _check_connection_and_readiness method to check for the connection status of subscribers, publishers 
        and services
        """
        self._check_connection_and_readiness()

        """
        initialise controller and sensor objects here
        """

        self.joint_names = ["elbow",
                            "gripper",
                            "left_finger",
                            "right_finger",
                            "shoulder",
                            "waist",
                            "wrist_angle",
                            "wrist_rotate"]

        # if self.real_time:
        #     # we don't need to pause/unpause gazebo if we are running in real time
        #     self.move_RX200_object = MoveitMultiros(arm_name='interbotix_arm',
        #                                             gripper_name='interbotix_gripper',
        #                                             robot_description="rx200/robot_description",
        #                                             ns="rx200", pause_gazebo=False)
        # else:
        #     self.move_RX200_object = MoveitMultiros(arm_name='interbotix_arm',
        #                                             gripper_name='interbotix_gripper',
        #                                             robot_description="rx200/robot_description",
        #                                             ns="rx200")

        """
        Finished __init__ method
        """
        if not self.real_time:
            gazebo_core.pause_gazebo()
        else:
            gazebo_core.unpause_gazebo()  # this is because loading models will pause the simulation
        rospy.loginfo("End Init RX200RobotEnv")

    # ---------------------------------------------------
    #   Custom methods for the Custom Robot Environment

    """
    Define the custom methods for the environment
        * joint_state_callback: Get the joint state of the robot
        * set_trajectory_joints: Set a joint position target only for the arm joints.
        * set_trajectory_ee: Set a pose target for the end effector of the robot arm.
        * get_ee_pose: Get end-effector pose a geometry_msgs/PoseStamped message
        * get_ee_rpy: Get end-effector orientation as a list of roll, pitch, and yaw angles.
        * get_joint_angles: Get current joint angles of the robot arm - 5 elements
        * check_goal: Check if the goal is reachable
    """

    def joint_state_callback(self, joint_state):
        """
        Function to get the joint state of the robot.
        """

        self.joint_state = joint_state
        # get the current
        self.current_joint_positions = joint_state.

    def set_trajectory_joints(self, q_positions: np.ndarray) -> bool:
        """
        Set a joint position target only for the arm joints.
        """

        if self.real_time:
            # do not wait for the action to finish
            return self.move_RX200_object.set_trajectory_joints(q_positions, async_move=True)
        else:
            return self.move_RX200_object.set_trajectory_joints(q_positions)

    def set_trajectory_ee(self, pos: np.ndarray) -> bool:
        """
        Set a pose target for the end effector of the robot arm.
        """
        if self.real_time:
            # do not wait for the action to finish
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

    # helper fn for _check_connection_and_readiness
    def _check_joint_states_ready(self):
        """
        Function to check if the joint states are received
        """
        if not self.real_time:
            gazebo_core.unpause_gazebo()  # Unpause Gazebo physics

        # Wait for the service to be available
        rospy.logdebug(rostopic.get_topic_type(self.joint_state_topic, blocking=True))

        return True

    # helper fn for _check_connection_and_readiness
    def _check_moveit_ready(self):
        """
        Function to check if moveit services are running
        """
        rospy.wait_for_service("/rx200/move_group/trajectory_execution/set_parameters")
        rospy.logdebug(rostopic.get_topic_type("/rx200/planning_scene", blocking=True))
        rospy.logdebug(rostopic.get_topic_type("/rx200/move_group/status", blocking=True))

        return True

    def _check_connection_and_readiness(self):
        """
        Function to check the connection status of subscribers, publishers and services, as well as the readiness of
        all systems.
        """
        self._check_moveit_ready()
        self._check_joint_states_ready()

        rospy.loginfo("All system are ready!")

        return True

    # ---------------------------------------------------
    #    Methods to override in Custom Task Environment

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
