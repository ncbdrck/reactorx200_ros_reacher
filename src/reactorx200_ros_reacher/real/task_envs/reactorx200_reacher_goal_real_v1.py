#!/bin/python3

import rospy
import numpy as np
from gym import spaces
from gym.envs.registration import register

import scipy.spatial

# Custom robot env
from reactorx200_ros_reacher.real.robot_envs import reactorx200_robot_goal_real_v1

# core modules of the framework
# from ros_rl.utils.moveit_ros_rl import MoveitROS_RL
from ros_rl.utils import ros_common
# from ros_rl.utils import ros_controllers
from ros_rl.utils import ros_markers

# Register your environment using the OpenAI register method to utilize gym.make("MyTaskGoalEnv-v0").
register(
    id='RX200ReacherGoalEnvReal-v1',
    entry_point='reactorx200_ros_reacher.real.task_envs.reactorx200_reacher_goal_real_v1:RX200ReacherGoalEnv',
    max_episode_steps=1000,
)

"""
This is the v1 of the RX200 Reacher Task Environment. Following are the new features of this environment:
    * Mainly with Moveit
    * Added support for Real time RL environment
    * We have new parameters for the environment - real_time, environment_loop_rate, action_cycle_time
    * Changed the observation - now we have the velocity of each joint as well as previous actions as part of the obs
    * We have a new method called environment_loop() for real time RL environments. This method is called by a timer so that we can run the environment at a given rate.
    * We also modified the methods _set_action(), _get_observation(), _get_reward() and _is_done() to support real time
    * changed the task config file to yaml
"""


class RX200ReacherGoalEnv(reactorx200_robot_goal_real_v1.RX200RobotGoalEnv):
    """
    This Task env is for a simple Reach Task with the RX200 robot.

    The task is done if
        * The robot reached the goal

    Here
        * Action Space - Continuous (5 actions or 3 actions) - Joint positions or x,y,z position of the EE
        * Observation - Continuous (28 obs or 26 obs) - EE pos, Vector to the goal, Euclidian distance, Joint values, Previous action, Joint velocities
        * Desired Goal - Goal we are trying to reach
        * Achieved Goal - Position of the EE

    Init Args:
        * launch_gazebo: Whether to launch Gazebo or not. If False, it is assumed that Gazebo is already running.
        * roscore_port: Port of the roscore to be launched. If None, a random port is chosen.
        * seed: Seed for the random number generator.
        * close_env_prompt: Whether to prompt the user to close the env or not.
        * reward_type: Type of reward to be used. Can be "Sparse" or "Dense".
        * ee_action_type: Whether to use the end-effector action space or the joint action space.
        * delta_action: Whether to use the delta actions or the absolute actions.
        * delta_coeff: Coefficient to be used for the delta actions.
        * real_time: Whether to use real time or not. (Asynchronous instead of sequential)
        * environment_loop_rate: Rate at which the environment loop should run. (in Hz)
        * action_cycle_time: Time to wait between two consecutive actions. (in seconds)
        * use_smoothing: Whether to use smoothing for actions or not.
        * default_port: Whether to use the default port for the roscore or not.
    """

    def __init__(self, new_roscore: bool = False, roscore_port: str = None, seed: int = None,
                 close_env_prompt: bool = True, reward_type: str = "sparse", ee_action_type: bool = False,
                 delta_action: bool = True, delta_coeff: float = 0.05, real_time: bool = False,
                 environment_loop_rate: float = None, action_cycle_time: float = 0.0,
                 use_smoothing: bool = False, default_port=True):

        """
        variables to keep track of ros port
        """
        ros_port = None

        """
        Initialise the env
        """

        # launch a new roscore with default port
        if default_port:
            ros_port = self._launch_roscore(default_port=default_port)

        # Launch new roscore
        elif new_roscore:
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
            self.node_name = "RX200ReacherGoalEnvReal" + "_" + ros_port
        else:
            self.node_name = "RX200ReacherGoalEnvReal"

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
        Use smoothing for actions
        """
        self.use_smoothing = use_smoothing
        self.action_cycle_time = action_cycle_time

        """
        Real time parameters
        """
        self.real_time = real_time

        # for moveit
        if real_time:
            async_moveit = True
        else:
            async_moveit = False

        """
        Load YAML param file
        """

        # add to ros parameter server
        ros_common.ros_load_yaml(pkg_name="reactorx200_ros_reacher", file_name="reach_task_config_v1.yaml", ns="/")
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
        04. Current Joint values - 8
        05. Previous action - 5 or 3 (joint or ee)
        06. Joint velocities - 8

        So observation space is a dictionary with 
            observation: (3x2) + 1 + 5 (or 3) + (8x2) = 28 or 26 elements
            achieved_goal: EE pos - 3 elements
            desired_goal: Goal pos - 3 elements
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

        # ---- joint values
        observations_high_joint_values = self.max_joint_angles.copy()
        observations_low_joint_values = self.min_joint_angles.copy()

        # ---- previous action
        if self.ee_action_type:
            observations_high_prev_action = self.max_ee_values.copy()
            observations_low_prev_action = self.min_ee_values.copy()
        else:
            observations_high_prev_action = self.max_joint_values.copy()
            observations_low_prev_action = self.min_joint_values.copy()

        # ---- joint velocities
        observations_high_joint_vel = self.max_joint_vel.copy()
        observations_low_joint_vel = self.min_joint_vel.copy()

        high = np.concatenate(
            [observations_high_ee_pos_range, observations_high_vec_ee_goal, observations_high_dist,
             observations_high_joint_values, observations_high_prev_action, observations_high_joint_vel, ])

        low = np.concatenate(
            [observations_low_ee_pos_range, observations_low_vec_ee_goal, observations_low_dist,
             observations_low_joint_values, observations_low_prev_action, observations_low_joint_vel, ])

        """
        Achieved goal (EE pose) - 3 
        """
        high_achieved_goal_pos_range = np.array(
            np.array([self.position_achieved_goal_max["x"], self.position_achieved_goal_max["y"],
                      self.position_achieved_goal_max["z"]]))
        low_achieved_goal_pos_range = np.array(
            np.array([self.position_achieved_goal_min["x"], self.position_achieved_goal_min["y"],
                      self.position_achieved_goal_min["z"]]))

        self.achieved_goal_space = spaces.Box(low=low_achieved_goal_pos_range, high=high_achieved_goal_pos_range,
                                              dtype=np.float32)

        """
        Desired goal (Goal pose) - 3 
        """
        high_desired_goal_pos_range = np.array(
            np.array([self.position_desired_goal_max["x"], self.position_desired_goal_max["y"],
                      self.position_desired_goal_max["z"]]))
        low_desired_goal_pos_range = np.array(
            np.array([self.position_desired_goal_min["x"], self.position_desired_goal_min["y"],
                      self.position_desired_goal_min["z"]]))

        self.desired_goal_space = spaces.Box(low=low_desired_goal_pos_range, high=high_desired_goal_pos_range,
                                             dtype=np.float32)

        """
        Define the overall observation space
        """
        self.observation_space = spaces.Dict({
            'observation': spaces.Box(low=low, high=high, dtype=np.float32),
            'achieved_goal': self.achieved_goal_space,
            'desired_goal': self.desired_goal_space
        })

        """
        Goal space for sampling
        """
        high_goal_pos_range = np.array(
            np.array([self.position_goal_max["x"], self.position_goal_max["y"], self.position_goal_max["z"]]))
        low_goal_pos_range = np.array(
            np.array([self.position_goal_min["x"], self.position_goal_min["y"], self.position_goal_min["z"]]))

        # -- goal space for sampling
        self.goal_space = spaces.Box(low=low_goal_pos_range, high=high_goal_pos_range, dtype=np.float32, seed=seed)

        """
        Define subscribers/publishers and Markers as needed.
        """

        self.goal_marker = ros_markers.RosMarker(frame_id="world", ns="goal", marker_type=2, marker_topic="goal_pos",
                                                 lifetime=30.0)

        """
        Init super class.
        """
        super().__init__(ros_port=ros_port, seed=seed, close_env_prompt=close_env_prompt, async_moveit=async_moveit,
                         action_cycle_time=action_cycle_time)

        # for smoothing
        if self.use_smoothing:
            if self.ee_action_type:
                self.action_vector = np.zeros(3, dtype=np.float32)
            else:
                self.action_vector = np.zeros(5, dtype=np.float32)

        # set the initial parameters for real time envs
        if environment_loop_rate is not None and self.real_time:
            self.obs_r = None
            self.reward_r = None
            self.done_r = None
            self.info_r = {}
            self.current_action = None
            self.init_done = False  # we don't need to execute the loop until we reset the env
            self.prev_action = None

            # Debug
            self.loop_counter = 0
            self.action_counter = 0

            # create a timer to run the environment loop
            rospy.Timer(rospy.Duration(1.0 / environment_loop_rate), self.environment_loop)

        # for dense reward calculation
        self.action_not_in_limits = False
        self.lowest_z = self.position_goal_min["z"]
        self.movement_result = False
        self.within_goal_space = False

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

        # make the current action None to stop execution for real time envs and also stop the env loop
        if self.real_time:
            self.init_done = False  # we don't need to execute the loop until we reset the env
            self.current_action = None

        # reset the action vector
        if self.use_smoothing:
            if self.ee_action_type:
                self.action_vector = np.zeros(3, dtype=np.float32)
            else:
                self.action_vector = np.zeros(5, dtype=np.float32)

        # stop the robot and move to the Home
        self.move_RX200_object.stop_arm()
        result = self.move_RX200_object.set_trajectory_joints(self.init_pos)
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
        # We don't need this because we reset env just before we start the episode (but just in case)
        ee_pos_tmp = self.get_ee_pose()  # Get a geometry_msgs/PoseStamped msg
        self.ee_pos = np.array([ee_pos_tmp.pose.position.x, ee_pos_tmp.pose.position.y, ee_pos_tmp.pose.position.z])
        self.joint_values = self.get_joint_angles()

        # for dense reward calculation
        self.action_not_in_limits = False
        self.within_goal_space = True

        # We can start the environment loop now
        if self.real_time:
            rospy.loginfo("Start resetting the env loop!")
            self.prev_action = self.init_pos.copy()

            # init the real time variables
            self.obs_r = None
            self.reward_r = None
            self.done_r = None
            self.info_r = {}

            # debug
            self.loop_counter = 0
            self.action_counter = 0

            rospy.loginfo("Done resetting the env loop!")

            self.init_done = True

        rospy.loginfo("Initialising init params done--->")

    def _set_action(self, action):
        """
        Function to apply an action to the robot.

        Args:
            action: The action to be applied to the robot.
        """
        # real time env
        if self.real_time:
            rospy.loginfo(f"Applying real-time action---> {action}")

            self.current_action = action.copy()
            self.prev_action = action.copy()
            self.action_counter = 0  # reset the action counter

        # normal env- Sequential
        else:
            self.execute_action(action)

    def _get_observation(self):
        """
        Function to get an observation from the environment.

        Returns:
            An observation representing the current state of the environment.
        """
        # real time env
        if self.real_time:

            if self.obs_r is not None:
                obs = self.obs_r.copy()
            else:
                obs = None

        # normal env- Sequential
        else:
            obs = self.sample_observation()

        # incase we don't have an observation yet for real time envs
        if obs is None:
            obs = self.sample_observation()

        return obs.copy()

    def _get_achieved_goal(self):
        """
        Get the achieved goal from the environment.

        Returns:
            achieved_goal: EE position
        """
        return self.ee_pos.copy()

    def _get_desired_goal(self):
        """
        Get the desired goal from the environment.

        Returns:
            desired_goal: Reach Goal
        """
        return self.reach_goal.copy()

    def compute_reward(self, achieved_goal, desired_goal, info) -> float:
        """
        Compute the reward for achieving a given goal.

        Returns:
            reward (float): The reward for achieving the given goal.
        """
        # real time env
        if self.real_time:
            reward = self.reward_r

        # normal env- Sequential
        else:
            reward = self.calculate_reward(achieved_goal, desired_goal, info)

        # incase we don't have a reward yet for real time envs
        if reward is None:
            reward = self.calculate_reward(achieved_goal, desired_goal, info)

        return reward

    def _is_done(self):
        """
        Function to check if the episode is done.

        Returns:
            A boolean value indicating whether the episode has ended
            (e.g., because a goal has been reached or a failure condition has been triggered)
        """

        # real time env
        if self.real_time:
            done = self.done_r
            self.info = self.info_r  # we can use this to log the success rate in stable baselines3

        # normal env- Sequential
        else:
            done = self.check_if_done()

        # incase we don't have a done yet for real time envs
        if done is None:
            done = self.check_if_done()

        return done

    # -------------------------------------------------------
    #   Include any custom methods available for the MyTaskEnv class

    def environment_loop(self, event):
        """
        Function for Environment loop for real time RL envs
        """

        #  we don't need to execute the loop until we reset the env
        if self.init_done and self.current_action is not None:

            rospy.loginfo(f"Starting RL loop --->: {self.loop_counter}")
            self.loop_counter += 1

            # start with the observation, reward, done and info
            self.info_r = {}
            self.obs_r = self.sample_observation()
            self.reward_r = self.calculate_reward()
            self.done_r = self.check_if_done(real_time=True)

            # Apply the action
            # we need this if we're done with the task we can break the loop in above done check
            if self.current_action is not None:
                self.execute_action(self.current_action)
                rospy.loginfo(f"Executing action --->: {self.action_counter}")
                self.action_counter += 1
            else:
                self.move_RX200_object.stop_arm()  # stop the arm if there is no action

    def execute_action(self, action):
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
                # we can use smoothing using the action_cycle_time or delta_coeff
                if self.use_smoothing:
                    if self.action_cycle_time == 0.0:
                        # first derivative of the action
                        self.action_vector = self.action_vector + (self.delta_coeff * action)

                        # clip the action vector to be within -1 and 1
                        self.action_vector = np.clip(self.action_vector, -1, 1)

                        # add the action vector to the current ee pos
                        action = self.ee_pos + (self.action_vector * self.delta_coeff)

                    else:
                        # first derivative of the action
                        self.action_vector = self.action_vector + (self.action_cycle_time * action)

                        # clip the action vector to be within -1 and 1
                        self.action_vector = np.clip(self.action_vector, -1, 1)

                        # add the action vector to the current ee pos
                        action = self.ee_pos + (self.action_vector * self.action_cycle_time)

                else:
                    action = self.ee_pos + (action * self.delta_coeff)

            # check if the action is within the joint limits
            min_ee_values = np.array(self.min_ee_values)
            max_ee_values = np.array(self.max_ee_values)
            self.action_not_in_limits = np.any(action <= (min_ee_values + 0.0001)) or np.any(
                action >= (max_ee_values - 0.0001))

            # clip the action
            action = np.clip(action, self.min_ee_values, self.max_ee_values)

            # check if we can reach the goal and within the goal space
            if self.check_goal(action):

                # check if action is within the z limits
                if action[2] > self.lowest_z:
                    # execute the trajectory - EE
                    self.movement_result = self.set_trajectory_ee(action)
                    self.within_goal_space = True

                else:
                    rospy.logdebug(f"Set action failed for --->: {action}")
                    self.movement_result = False
                    self.within_goal_space = False

            else:
                rospy.logwarn(f"The action: {action} is not reachable!")
                rospy.logdebug(f"Set action failed for --->: {action}")
                self.movement_result = False
                self.within_goal_space = False

        # --- Joint action
        else:

            # --- Make actions as deltas
            if self.delta_action:
                # we can use smoothing using the action_cycle_time or delta_coeff
                if self.use_smoothing:
                    if self.action_cycle_time == 0.0:
                        # first derivative of the action
                        self.action_vector = self.action_vector + (self.delta_coeff * action)

                        # clip the action vector to be within -1 and 1
                        self.action_vector = np.clip(self.action_vector, -1, 1)

                        # add the action vector to the current ee pos
                        action = self.joint_values + (self.action_vector * self.delta_coeff)

                    else:
                        # first derivative of the action
                        self.action_vector = self.action_vector + (self.action_cycle_time * action)

                        # clip the action vector to be within -1 and 1
                        self.action_vector = np.clip(self.action_vector, -1, 1)

                        # add the action vector to the current ee pos
                        action = self.joint_values + (self.action_vector * self.action_cycle_time)

                else:
                    action = self.joint_values + (action * self.delta_coeff)

            # check if the action is within the joint limits
            min_joint_values = np.array(self.min_joint_values)
            max_joint_values = np.array(self.max_joint_values)
            self.action_not_in_limits = np.any(action <= (min_joint_values + 0.0001)) or np.any(
                action >= (max_joint_values - 0.0001))

            # clip the action
            action = np.clip(action, self.min_joint_values, self.max_joint_values)

            # check if we can reach the goal
            if self.check_goal_reachable_joint_pos(action):

                # check if the action is within the z limits
                if self.check_if_z_within_limits(action):
                    # execute the trajectory
                    self.movement_result = self.set_trajectory_joints(action)
                    self.within_goal_space = True

                else:
                    rospy.logdebug(f"Set action failed for --->: {action}")
                    self.movement_result = False
                    self.within_goal_space = False

            else:
                rospy.logwarn(f"The action: {action} is not reachable!")
                rospy.logdebug(f"Set action failed for --->: {action}")
                self.movement_result = False
                self.within_goal_space = False

    def sample_observation(self):
        """
        Function to get an observation from the environment.

        # observation
        01. EE pos - 3
        02. Vector to the goal (normalized linear distance) - 3
        03. Euclidian distance (ee to reach goal)- 1
        04. Current Joint values - 8
        05. Previous action - 5 or 3 (joint or ee)
        06. Joint velocities - 8

        total: (3x2) + 1 + (5 or 3)  + (8x2) = 28 or 26

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

        # --- Get Current Joint values - only for the joints we are using
        #  we need this for delta actions
        # self.joint_values = self.current_joint_positions.copy()  # Get a float list
        self.joint_values = self.get_joint_angles()  # Get a float list. For action as delta
        # we don't need to convert this to numpy array since we concat using numpy below

        # our observations
        obs = np.concatenate((self.ee_pos, vec_ee_goal, euclidean_distance_ee_goal, self.joint_pos_all,
                              self.prev_action, self.current_joint_velocities), axis=None)

        rospy.loginfo(f"Observations --->: {obs}")

        return obs.copy()

    def calculate_reward(self, achieved_goal=None, desired_goal=None, info=None):
        """
        Function to get a reward from the environment.

        Sparse Reward: float => 1.0 for success, -1.0 for failure

        Dense Reward:
            if reached: self.reached_goal_reward (positive reward)
            else: - self.mult_dist_reward * distance_to_the_goal

            and as always negative rewards for each step, non-execution and actions not within joint limits

        Args:
            achieved_goal: EE position (optional)
            desired_goal: Reach Goal (optional)
            info (dict): Additional information about the environment. (Optional)

        Returns:
            A scalar reward value representing how well the agent is doing in the current episode.
        """
        # - Init reward
        reward = 0

        achieved_goal = self.ee_pos
        desired_goal = self.reach_goal

        # if it's "Sparse" reward structure
        if self.reward_arc == "Sparse":

            # initialise the sparse reward as negative
            reward = -1

            # The marker only turns green if reach is done. Otherwise, it is red.
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

            # - Check if actions are in limits
            reward += self.action_not_in_limits * self.joint_limits_reward

            # - Check if the action is within the goal space
            reward += (not self.within_goal_space) * self.not_within_goal_space_reward

            # to punish for actions where we cannot execute
            if not self.movement_result:
                reward += self.none_exe_reward

            # log the reward
            rospy.logwarn(">>>REWARD>>>" + str(reward))

        return reward

    def check_if_done(self, real_time=False):
        """
        Function to check if the episode is done.

        The Task is done if the EE is close enough to the goal

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
            if real_time:
                self.info_r['is_success'] = 1.0
                self.current_action = None  # we don't need to execute any more actions
                self.init_done = False  # we don't need to execute the loop until we reset the env
            else:
                self.info['is_success'] = 1.0

        return done

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

    def check_action_within_goal_space_pykdl(self, action):
        """
        Function to check if the given action is within the goal space

        Args:
            action: The action to be applied to the robot.

        Returns:
            A boolean value indicating whether the action is within the goal space
        """

        BLUE = '\033[94m'
        ENDC = '\033[0m'

        # check if the resulting ee pose is within the goal space - using FK
        ee_pos = self.fk_pykdl(action=action)

        # print("goal space", self.goal_space)  # for debugging

        if ee_pos is not None:
            # check if the ee pose is within the goal space - using self.goal_space
            if self.goal_space.contains(ee_pos):
                rospy.logdebug(f"The ee pose {ee_pos} of the {action} is within the goal space!")
                # print(BLUE + f"The ee pose {ee_pos} of the {action} is within the goal space!" + ENDC)  # for debugging
                return True
            else:
                rospy.logdebug(f"The ee pose {ee_pos} is not within the goal space!")
                # print(f"The ee pose {ee_pos} is not within the goal space!")  # for debugging
                return False
        else:
            rospy.logwarn("Checking if the action is within the goal space failed!")
            # print("Checking if the action is within the goal space failed!")  # for debugging
            return False

    def check_if_z_within_limits(self, action):
        """
        Function to check if the given ee_pos is within the limits
        """
        # get the ee pose from the action using FK
        ee_pos = self.fk_pykdl(action=action)

        # The robot is mounted on a table. So, we need to check if the z is within the limits
        if ee_pos is not None:

            # check if the z is within the limits
            if ee_pos[2] > self.lowest_z:
                rospy.logdebug(f"The ee pose {ee_pos} of the {action} is within the z limit!")
                return True
            else:
                rospy.logdebug(f"The ee pose {ee_pos} is not within the z limit!")
                return False
        else:
            rospy.logwarn("Checking if the action is within the z limit failed!")
            return False

    def _get_params(self):
        """
        Function to get configuration parameters (optional)
        """
        # Task Related parameters (we don't need these)
        if self.ee_action_type:
            self.n_actions = rospy.get_param('/rx200/n_actions_ee')
            self.n_observations = rospy.get_param('/rx200/n_observations_ee')
        else:
            self.n_actions = rospy.get_param('/rx200/n_actions_joint')
            self.n_observations = rospy.get_param('/rx200/n_observations_joint')

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
        self.min_joint_vel = rospy.get_param('/rx200/min_joint_vel')
        self.max_joint_vel = rospy.get_param('/rx200/max_joint_vel')
        self.min_joint_angles = rospy.get_param('/rx200/min_joint_angles')
        self.max_joint_angles = rospy.get_param('/rx200/max_joint_angles')

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
        self.not_within_goal_space_reward = rospy.get_param('/rx200/not_within_goal_space_reward')

    # ------------------------------------------------------
    #   Task Methods for launching roscore

    @staticmethod
    def _launch_roscore(port=None, set_new_master_vars=False, default_port=False):
        """
        Launches a new roscore with the specified port. Only updates the ros_port.

        Return:
            ros_port: port of launched roscore
        """

        if port is None:
            port = int(port)

        ros_port = ros_common.launch_roscore(port=port, set_new_master_vars=set_new_master_vars,
                                             default_port=default_port)

        # change to new rosmaster
        ros_common.change_ros_master(ros_port)

        return ros_port
