#!/bin/python3

import rospy
import numpy as np
from gym import spaces
from gym.envs.registration import register

import scipy.spatial

# Custom robot env
from reactorx200_ros_reacher.real.robot_envs import reactorx200_robot_real

# core modules of the framework
from ros_rl.utils import ros_common
# from ros_rl.utils.moveit_ros_rl import MoveitROS_RL
# from ros_rl.utils import ros_controllers
from ros_rl.utils import ros_markers

# Register your environment using the OpenAI register method to utilize gym.make("TaskEnv-v0").
# register(
#     id='RX200ReacherEnvReal-v0',
#     entry_point='reactorx200_ros_reacher.real.task_envs.reactorx200_reacher_real:RX200ReacherEnv',
#     max_episode_steps=100,
# )


class RX200ReacherEnv(reactorx200_robot_real.RX200RobotEnv):
    """
    This Task env is for a simple Reach Task with the RX200 robot.

    The task is done if
        * The robot reached the goal

    Here
        * Action Space - Continuous (5 actions or 3 actions) - Joint positions or x,y,z position of the EE
        * Observation  - Continuous (12 obs)

    Init Args:
        * new_roscore: Whether to launch a new roscore or not. If False, it is assumed that a roscore is already running.
        * roscore_port: Port of the roscore to be launched. If None, a random port is chosen.
        * seed: Seed for the random number generator.
        * close_env_prompt: Whether to prompt the user to close the env or not.
        * reward_type: Type of reward to be used. Can be "Sparse" or "Dense".
        * ee_action_type: Whether to use the end-effector action space or the joint action space.
        * delta_action: Whether to use the delta actions or the absolute actions.
        * delta_coeff: Coefficient to be used for the delta actions.
    """

    def __init__(self, new_roscore: bool = False, roscore_port: str = None, seed: int = None,
                 close_env_prompt: bool = True, reward_type: str = "Dense", ee_action_type: bool = False,
                 delta_action: bool = False, delta_coeff: float = 0.05):

        """
        variables to keep track of ros port
        """
        ros_port = None

        """
        Initialise the env
        """

        # Launch new roscore
        if new_roscore:
            ros_port = self._launch_roscore(port=roscore_port)

        # ros_port of the already running roscore
        elif roscore_port is not None:
            ros_port = roscore_port

            # change to new rosmaster
            ros_common.change_ros_master(ros_port)

        else:
            """
            Check for roscore
            """
            if ros_common.is_roscore_running() is False:
                print("roscore is not running! Launching a new roscore!")
                ros_port = self._launch_roscore(port=roscore_port)

        # init the ros node
        if ros_port is not None:
            self.node_name = "RX200ReacherEnvReal" + "_" + ros_port
        else:
            self.node_name = "RX200ReacherEnvReal"

        rospy.init_node(self.node_name, anonymous=True)

        """
        Provide a description of the task.
        """
        rospy.loginfo(f"Starting {self.node_name}")

        """
        Reward Architecture
            * Dense - Default
            * Sparse - -1 if not done and 1 if done
            * All the others and misspellings - default to "Dense" reward
        """
        if reward_type.lower() == "sparse":
            self.reward_arc = "Sparse"

        elif reward_type.lower() == "dense":
            self.reward_arc = "Dense"

        else:
            rospy.logwarn(f"The given reward architecture '{reward_type}' not found. Defaulting to Dense!")
            self.reward_arc = "Dense"

        """
        Action type
            * Joints - Default
            * EE - End Effector
        """
        self.ee_action_type = ee_action_type

        """
        Use action as deltas
        """
        self.delta_action = delta_action
        self.delta_coeff = delta_coeff

        """
        Load YAML param file
        """

        # add to ros parameter server
        ros_common.ros_load_yaml(pkg_name="reactorx200_ros_reacher", file_name="reach_task_config.yaml", ns="/")
        self._get_params()

        """
        Define the action space.
        """
        if self.ee_action_type:
            # EE action space

            self.max_ee_values = np.array([self.position_ee_max["x"], self.position_ee_max["y"],
                                           self.position_ee_max["z"]])
            self.min_ee_values = np.array([self.position_ee_min["x"], self.position_ee_min["y"],
                                           self.position_ee_min["z"]])

            self.action_space = spaces.Box(low=np.array(self.min_ee_values), high=np.array(self.max_ee_values),
                                           dtype=np.float32)

        else:
            # Joint action space
            # ROS and Gazebo often use double-precision (64-bit)
            # but we are using single-precision (32-bit) for our action space because we are using stable baselines3
            self.action_space = spaces.Box(low=np.array(self.min_joint_values), high=np.array(self.max_joint_values),
                                           dtype=np.float32)

        """
        Define the observation space.
        
        # observation
        01. EE pos - 3
        02. Vector to the goal (normalized linear distance) - 3
        03. Euclidian distance (ee to reach goal)- 1
        04. Current Joint values - 5

        total: (3x2) + 1 + 5 = 12
        """

        # ---- ee pos
        observations_high_ee_pos_range = np.array(
            np.array([self.position_ee_max["x"], self.position_ee_max["y"], self.position_ee_max["z"]]))
        observations_low_ee_pos_range = np.array(
            np.array([self.position_ee_min["x"], self.position_ee_min["y"], self.position_ee_min["z"]]))

        # ---- vector to the goal - normalized linear distance
        observations_high_vec_ee_goal = np.array([1.0, 1.0, 1.0])
        observations_low_vec_ee_goal = np.array([-1.0, -1.0, -1.0])

        # ---- Euclidian distance
        observations_high_dist = np.array([self.max_distance])
        observations_low_dist = np.array([0.0])

        high = np.concatenate(
            [observations_high_ee_pos_range, observations_high_vec_ee_goal, observations_high_dist,
             self.max_joint_values, ])

        low = np.concatenate(
            [observations_low_ee_pos_range, observations_low_vec_ee_goal, observations_low_dist,
             self.min_joint_values, ])

        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

        """
        Goal space for sampling
        """
        # ---- Goal pos
        high_goal_pos_range = np.array(
            np.array([self.position_goal_max["x"], self.position_goal_max["y"], self.position_goal_max["z"]]))
        low_goal_pos_range = np.array(
            np.array([self.position_goal_min["x"], self.position_goal_min["y"], self.position_goal_min["z"]]))

        # -- goal space for sampling
        self.goal_space = spaces.Box(low=low_goal_pos_range, high=high_goal_pos_range, dtype=np.float32)

        """
        Define subscribers/publishers and Markers as needed.
        """

        self.goal_marker = ros_markers.RosMarker(frame_id="world", ns="goal", marker_type=2, marker_topic="goal_pos",
                                                 lifetime=30.0)

        """
        Init super class.
        """
        super().__init__(ros_port=ros_port, seed=seed, close_env_prompt=close_env_prompt)

        """
        Finished __init__ method
        """
        rospy.loginfo(f"Finished Init of {self.node_name}")

    # -------------------------------------------------------
    #   Methods for interacting with the environment

    def _set_init_params(self):
        """
        Set initial parameters for the environment.

        Here we
            1. Move the Robot to Home
            2. Find a valid random reach goal

        """
        rospy.loginfo("Initialising the init params!")

        # Initial robot pose - Home
        self.init_pos = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # move the robot to home
        result = self.set_trajectory_joints(self.init_pos)
        if not result:
            rospy.logwarn("Homing failed!")

        #  Get a random Reach goal - np.array
        goal_found, goal_vector = self.get_random_goal()

        if goal_found:
            self.reach_goal = goal_vector
            rospy.loginfo("Reach Goal--->" + str(self.reach_goal))

        else:
            # fake Reach goal - hard code one
            self.reach_goal = np.array([0.250, 0.000, 0.015], dtype=np.float32)
            rospy.logwarn("Hard Coded Reach Goal--->" + str(self.reach_goal))

        # Publish the goal pos
        self.goal_marker.set_position(position=self.reach_goal)
        self.goal_marker.publish()

        # get initial ee pos and joint values (we need this for delta actions)
        # we don't need this because we reset env just before we start the episode (but just incase)
        self.ee_pos = self.get_ee_pose()
        self.joint_values = self.get_joint_angles()

        rospy.loginfo("Initialising init params done--->")


    def _set_action(self, action):
        """
        Function to apply an action to the robot.

        This method should be implemented here to apply the given action to the robot. The action could be a
        joint position command, a velocity command, or any other type of command that can be applied to the robot.

        Args:
            action: The action to be applied to the robot.
        """
        rospy.loginfo(f"Action --->: {action}")

        # --- Set the action based on the action type
        # --- EE action
        if self.ee_action_type:

            # --- Make actions as deltas
            if self.delta_action:
                action = self.ee_pos + (action * self.delta_coeff)

            # clip the action
            action = np.clip(action, self.min_ee_values, self.max_ee_values)

            # execute the trajectory
            self.movement_result = self.set_trajectory_ee(action)

        # --- Joint action
        else:

            # --- Make actions as deltas
            if self.delta_action:
                action = self.joint_values + (action * self.delta_coeff)

            # clip the action
            # rospy.logwarn(f"Action + current joint_values before clip --->: {action}")
            action = np.clip(action, self.min_joint_values, self.max_joint_values)
            # rospy.logwarn(f"Action after --->: {action}")

            # execute the trajectory
            self.movement_result = self.set_trajectory_joints(action)

        if not self.movement_result:
            rospy.logwarn(f"Set action failed for --->: {action}")
        else:
            rospy.logdebug(f"Movement was successful for --->: {action}")

    def _get_observation(self):
        """
        Function to get an observation from the environment.

        # observation
        01. EE pos - 3
        02. Vector to the goal (normalized linear distance) - 3
        03. Euclidian distance (ee to reach goal)- 1
        04. Current Joint values - 5

        total: (3x2) + 1 + 5 = 12

        Returns:
            An observation representing the current state of the environment.
        """
        current_goal = self.reach_goal

        # --- 1. Get EE position
        ee_pos_tmp = self.get_ee_pose()  # Get a geometry_msgs/PoseStamped msg
        self.ee_pos = np.array([ee_pos_tmp.pose.position.x, ee_pos_tmp.pose.position.y, ee_pos_tmp.pose.position.z])

        # --- Linear distance to the goal
        linear_dist_ee_goal = current_goal - self.ee_pos  # goal is box dtype and ee_pos is numpy.array. It is okay

        # --- 2. Vector to goal (we are giving only the direction vector)
        vec_ee_goal = linear_dist_ee_goal / np.linalg.norm(linear_dist_ee_goal)

        # --- 3. Euclidian distance
        euclidean_distance_ee_goal = scipy.spatial.distance.euclidean(self.ee_pos, current_goal)  # float

        # --- 4. Get Current Joint values
        self.joint_values = self.get_joint_angles()  # Get a list
        # we don't need to convert this to numpy array since we concat using numpy below

        # our observations
        obs = np.concatenate((self.ee_pos, vec_ee_goal, euclidean_distance_ee_goal, self.joint_values), axis=None)

        rospy.loginfo(f"Observations --->: {obs}")

        return obs.copy()

    def _get_reward(self):
        """
        Function to get a reward from the environment.

        Sparse Reward: float => 1.0 for success, -1.0 for failure

        Dense Reward:
            if reached: self.reached_goal_reward (positive reward)
            else: - self.mult_dist_reward * distance_to_the_goal

            and as always negative rewards for each step, non execution and actions not within joint limits

        Returns:
            A scalar reward value representing how well the agent is doing in the current episode.
        """
        # - Init reward
        reward = 0

        achieved_goal = self.ee_pos
        desired_goal = self.reach_goal

        # if it's "Sparse" reward structure
        if self.reward_arc == "Sparse":

            # initialize the sparse reward as negative
            reward = -1

            # marker only turns green if reach is done. Otherwise, it is red.
            self.goal_marker.set_color(r=1.0, g=0.0)
            self.goal_marker.set_duration(duration=5)

            # check if robot reached the goal
            reach_done = self.check_if_reach_done(achieved_goal, desired_goal)

            if reach_done:
                reward = 1

                # done (green) goal_marker
                self.goal_marker.set_color(r=0.0, g=1.0)

            # publish the marker to the topic
            self.goal_marker.publish()

            # log the reward
            rospy.logwarn(">>>REWARD>>>" + str(reward))

        # Since we only look for Sparse or Dense, we don't need to check if it's Dense
        else:
            # - Check if the EE reached the goal
            done = self.check_if_reach_done(achieved_goal, desired_goal)

            if done:
                # EE reached the goal
                reward += self.reached_goal_reward

                # done (green) goal_marker
                self.goal_marker.set_color(r=0.0, g=1.0)
                self.goal_marker.set_duration(duration=5)

            else:
                # not done (red) goal_marker
                self.goal_marker.set_color(r=1.0, g=0.0)
                self.goal_marker.set_duration(duration=5)

                # - Distance from EE to goal reward
                dist2goal = scipy.spatial.distance.euclidean(achieved_goal, desired_goal)
                reward += - self.mult_dist_reward * dist2goal

                # - Constant step reward
                reward += self.step_reward

            # publish the goal marker
            self.goal_marker.publish()

            # - Check if joints are in limits
            joint_angles = np.array(self.joint_values)
            min_joint_values = np.array(self.min_joint_values)
            max_joint_values = np.array(self.max_joint_values)
            in_limits = np.any(joint_angles <= (min_joint_values + 0.0001)) or np.any(
                joint_angles >= (max_joint_values - 0.0001))
            reward += in_limits * self.joint_limits_reward

            # to punish for actions where we cannot execute
            if not self.movement_result:
                reward += self.none_exe_reward

            # log the reward
            rospy.logwarn(">>>REWARD>>>" + str(reward))

        return reward

    def _is_done(self):
        """
        Function to check if the episode is done.

        Task is done if the EE is close enough to the goal

        Returns:
            A boolean value indicating whether the episode has ended
            (e.g., because a goal has been reached or a failure condition has been triggered)
        """
        # this is for logging in different colours
        # Define ANSI escape codes for different colors
        RED = '\033[91m'
        GREEN = '\033[92m'
        YELLOW = '\033[93m'
        BLUE = '\033[94m'
        ENDC = '\033[0m'

        # --- Init done
        done = False

        # - Check if the ee reached the goal
        done_reach = self.check_if_reach_done(self.ee_pos, self.reach_goal)

        if done_reach:
            rospy.loginfo(GREEN + ">>>>>>>>>>>> Reached the Goal! >>>>>>>>>>>" + ENDC)
            done = True

            # we can use this to log the success rate in stable baselines3
            self.info['is_success'] = 1.0

        return done

    # -------------------------------------------------------
    #   Include any custom methods available for the MyTaskEnv class

    def check_if_reach_done(self, achieved_goal, desired_goal):
        """
        Check if the reach is done
        """
        done = False

        # distance between achieved goal and desired goal
        distance = scipy.spatial.distance.euclidean(achieved_goal, desired_goal)
        # print("distance to the goal", distance)

        if distance <= self.reach_tolerance:
            done = True

        return done

    def test_goal_pos(self, goal):
        """
        Function to check if the given goal is reachable
        """
        rospy.logdebug(f"Goal to check: {str(goal)}")
        result = self.check_goal(goal)

        if not result:
            rospy.logdebug("The goal is not reachable")

        return result

    def get_random_goal(self, max_tries: int = 100):
        """
        Function to get a reachable goal
        """
        for i in range(max_tries):
            goal = self.goal_space.sample()

            if self.test_goal_pos(goal):
                return True, goal

        rospy.logdebug("Getting a random goal failed!")

        return False, None

    def _get_params(self):
        """
        Function to get configuration parameters (optional)
        """

        # Task Related parameters (we don't need these)
        if self.ee_action_type:
            self.n_actions = rospy.get_param('/rx200/n_actions_ee')
        else:
            self.n_actions = rospy.get_param('/rx200/n_actions_joint')

        self.n_observations = rospy.get_param('/rx200/n_observations')

        # Action Space
        self.min_joint_values = rospy.get_param('/rx200/min_joint_pos')
        self.max_joint_values = rospy.get_param('/rx200/max_joint_pos')

        # Observation Space
        self.position_ee_max = rospy.get_param('/rx200/position_ee_max')
        self.position_ee_min = rospy.get_param('/rx200/position_ee_min')
        self.rpy_ee_max = rospy.get_param('/rx200/rpy_ee_max')
        self.rpy_ee_min = rospy.get_param('/rx200/rpy_ee_min')
        self.linear_distance_max = rospy.get_param('/rx200/linear_distance_max')
        self.linear_distance_min = rospy.get_param('/rx200/linear_distance_min')
        self.max_distance = rospy.get_param('/rx200/max_distance')

        # Goal space
        self.position_goal_max = rospy.get_param('/rx200/position_goal_max')
        self.position_goal_min = rospy.get_param('/rx200/position_goal_min')

        # Achieved goal
        self.position_achieved_goal_max = rospy.get_param('/rx200/position_achieved_goal_max')
        self.position_achieved_goal_min = rospy.get_param('/rx200/position_achieved_goal_min')

        # Desired goal
        self.position_desired_goal_max = rospy.get_param('/rx200/position_desired_goal_max')
        self.position_desired_goal_min = rospy.get_param('/rx200/position_desired_goal_min')

        # Tolerances
        self.reach_tolerance = rospy.get_param('/rx200/reach_tolerance')

        # Variables related to rewards
        self.step_reward = rospy.get_param('/rx200/step_reward')
        self.mult_dist_reward = rospy.get_param('/rx200/multiplier_dist_reward')
        self.reached_goal_reward = rospy.get_param('/rx200/reached_goal_reward')
        self.joint_limits_reward = rospy.get_param('/rx200/joint_limits_reward')
        self.none_exe_reward = rospy.get_param('/rx200/none_exe_reward')

    # ------------------------------------------------------
    #   Task Methods for launching roscore

    def _launch_roscore(self, port=None, set_new_master_vars=False):
        """
        Launches a new roscore with the specified port. Only updates the ros_port.

        Return:
            ros_port: port of launched roscore
        """

        ros_port, _ = ros_common.launch_roscore(port=int(port), set_new_master_vars=set_new_master_vars)

        # change to new rosmaster
        ros_common.change_ros_master(ros_port)

        return ros_port
