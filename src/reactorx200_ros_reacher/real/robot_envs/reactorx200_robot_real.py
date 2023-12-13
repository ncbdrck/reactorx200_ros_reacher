#!/bin/python3

import rospy
import rostopic
from gym import spaces
from gym.envs.registration import register

import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from realros.envs import RealBaseEnv

# core modules of the framework
from realros.utils.moveit_realros import MoveitRealROS
from realros.utils import ros_common
from realros.utils import ros_controllers
from realros.utils import ros_markers

"""
Although it is best to register only the task environment, one can also register the robot environment. 
This is not necessary, but we can see if this section works by calling "gym.make" this env.
"""
register(
    id='RX200RobotBaseEnv-v0',
    entry_point='reactorx200_ros_reacher.real.robot_envs.reactorx200_robot_real:RX200RobotEnv',
    max_episode_steps=100,
)


class RX200RobotEnv(RealBaseEnv.RealBaseEnv):
    """
    RX200 Robot Env, use this class to describe the robots and the sensors in the Environment.
    Superclass for all Robot environments.
    """

    def __init__(self, ros_port: str = None, seed: int = None, close_env_prompt: bool = False):
        """
        Initializes a new Robot Environment

        Describe the robot and the sensors used in the env.

        Sensor Topic List:
            /joint_states : JointState received for the joints of the robot

        Actuators Topic List:
            MoveIt! : MoveIt! action server is used to send the joint positions to the robot.
        """
        rospy.loginfo("Start Init RX200RobotEnv RealROS")

        """
        Change the ros/gazebo master
        """
        if ros_port is not None:
            ros_common.change_ros_master(ros_port=ros_port)

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
            ros_port=ros_port, seed=seed, close_env_prompt=close_env_prompt)

        """
        Using the _check_connection_and_readiness method to check for the connection status of subscribers, publishers 
        and services
        """
        self._check_connection_and_readiness()

        """
        initialise controller and sensor objects here
        """

        # Moveit object
        self.move_RX200_object = MoveitRealROS(arm_name='interbotix_arm',
                                               gripper_name='interbotix_gripper',
                                               robot_description="rx200/robot_description",
                                               ns="rx200")

        """
        Finished __init__ method
        """
        rospy.loginfo("End Init RX200RobotEnv")

    # ---------------------------------------------------
    #   Custom methods for the Robot Environment

    """
    Define the custom methods for the environment
        * set_trajectory_joints: Set a joint position target only for the arm joints.
        * set_trajectory_ee: Set a pose target for the end effector of the robot arm.
        * get_ee_pose: Get end-effector pose a geometry_msgs/PoseStamped message
        * get_ee_rpy: Get end-effector orientation as a list of roll, pitch, and yaw angles.
        * get_joint_angles: Get current joint angles of the robot arm - 5 elements
        * check_goal: Check if the goal is reachable
    """

    def set_trajectory_joints(self, q_positions: np.ndarray) -> bool:
        """
        Set a joint position target only for the arm joints.
        """
        return self.move_RX200_object.set_trajectory_joints(q_positions)

    def set_trajectory_ee(self, pos: np.ndarray) -> bool:
        """
        Set a pose target for the end effector of the robot arm.
        """
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

    def _check_moveit_ready(self):
        """
        Function to check if moveit services are running
        """
        rospy.wait_for_service("/rx200/move_group/trajectory_execution/set_parameters")
        rospy.logdebug(rostopic.get_topic_type("/rx200/planning_scene", blocking=True))
        rospy.logdebug(rostopic.get_topic_type("/rx200/move_group/status", blocking=True))

        return True

    # ---------------------------------------------------
    #   Methods to override in the Robot Environment

    def _check_connection_and_readiness(self):
        """
        Function to check the connection status of subscribers, publishers and services, as well as the readiness of
        all systems.
        """
        self._check_moveit_ready()

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
