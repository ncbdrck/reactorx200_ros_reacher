#!/bin/python3
import sys

# ROS packages required
import rospy

# gym
import gym
import numpy as np

# We can use the following import statement if we want to use the ros_rl package
from ros_rl.core import ros_rl_gym
from ros_rl.utils import ros_common

# Environments are registered inside the main __init__.py of the reactorx200_ros_reacher package
import reactorx200_ros_reacher

# Models
from sb3_ros_support.sac import SAC
from sb3_ros_support.td3 import TD3
from sb3_ros_support.td3_goal import TD3_GOAL

# wrappers
from ros_rl.wrappers.normalize_action_wrapper import NormalizeActionWrapper
from ros_rl.wrappers.normalize_obs_wrapper import NormalizeObservationWrapper
from ros_rl.wrappers.time_limit_wrapper import TimeLimitWrapper

if __name__ == '__main__':
    # Kill all processes related to previous runs
    # ros_common.kill_all_ros_processes()

    # Clear ROS logs
    # ros_common.clean_ros_logs()

    # --- normal environments
    # env = ros_rl_gym.make('RX200ReacherEnvReal-v0', ee_action_type=False, delta_action=False)

    env = gym.make('RX200ReacherEnvSim-v2',  delta_action=True, real_time=True, reward_type="dense",
                   environment_loop_rate=50.0, action_cycle_time=0.2, seed=10)

    # --- goal-conditioned environments
    # env = ros_rl_gym.make('RX200ReacherGoalEnvReal-v0', ee_action_type=False, delta_action=False,
    #                       reward_type="sparse")

    # env = gym.make('RX200ReacherGoalEnvSim-v2', delta_action=True, reward_type="sparse", real_time=True,
    #                environment_loop_rate=50.0, action_cycle_time=0.2, seed=10)

    # Normalize action space
    env = NormalizeActionWrapper(env)

    # Normalize observation space
    env = NormalizeObservationWrapper(env)
    # env = NormalizeObservationWrapper(env, normalize_goal_spaces=True)  # goal-conditioned environments

    # Set max steps
    termination_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    # env = TimeLimitWrapper(env, max_steps=100, termination_action=termination_action)
    env = TimeLimitWrapper(env, max_steps=100)

    # reset the environment
    env.reset()

    # path to the package
    pkg_path = "reactorx200_ros_reacher"

    # # Default base environments - SAC
    # config_file_name = "sac.yaml"
    # save_path = "/models/real/sac/"
    # log_path = "/logs/real/sac/"
    #
    # # # create the model
    # model = SAC(env, save_path, log_path, model_pkg_path=pkg_path,
    #             config_file_pkg=pkg_path, config_filename=config_file_name)

    # Default base environments - TD3
    config_file_name = "td3.yaml"
    save_path = "/models/real/td3/"
    log_path = "/logs/real/td3/"

    # create the model
    model = TD3(env, save_path, log_path, model_pkg_path=pkg_path,
                config_file_pkg=pkg_path, config_filename=config_file_name)

    # # Goal-conditioned environments - TD3+HER
    # config_file_name = "td3_goal.yaml"
    # save_path = "/models/real/td3_goal/"
    # log_path = "/logs/real/td3_goal/"

    # # create the model
    # model = TD3_GOAL(env, save_path, log_path, model_pkg_path=pkg_path,
    #                  config_file_pkg=pkg_path, config_filename=config_file_name)

    # train the models
    model.train()
    model.save_model()
    model.close_env()

    sys.exit()
