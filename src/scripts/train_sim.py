#!/bin/python3
import sys

# ROS packages required
import rospy

# gym
import gym
import numpy as np

# We can use the following import statement if we want to use the multiros package
from multiros.core import multiros_gym
from multiros.utils import ros_common

# Environments are registered inside the main __init__.py of the reactorx200_ros_reacher package
import reactorx200_ros_reacher
# or
# from reactorx200_ros_reacher.sim.task_envs import reactorx200_reacher_sim, reactorx200_reacher_goal_sim
# from reactorx200_ros_reacher.sim.task_envs import reactorx200_reacher_sim_v1, reactorx200_reacher_goal_sim_v1
# from reactorx200_ros_reacher.sim.task_envs import reactorx200_reacher_sim_v2, reactorx200_reacher_goal_sim_v2

# Models
from sb3_ros_support.sac import SAC
from sb3_ros_support.td3 import TD3
from sb3_ros_support.td3_goal import TD3_GOAL
from sb3_ros_support.dqn import DQN
from sb3_ros_support.dqn_goal import DQN_GOAL

# wrappers
from multiros.wrappers.normalize_action_wrapper import NormalizeActionWrapper
from multiros.wrappers.normalize_obs_wrapper import NormalizeObservationWrapper
from multiros.wrappers.time_limit_wrapper import TimeLimitWrapper

if __name__ == '__main__':
    # Kill all processes related to previous runs
    ros_common.kill_all_ros_and_gazebo()

    # Clear ROS logs
    # ros_common.clean_ros_logs()

    # --- normal environments
    # env = gym.make('RX200ReacherEnvSim-v1', gazebo_gui=False, ee_action_type=False,
    #                delta_action=True, real_time=True, environment_loop_rate=10.0, action_cycle_time=0.8,
    #                use_smoothing=True)

    # env = gym.make('RX200ReacherEnvSim-v2', gazebo_gui=False, delta_action=True, real_time=True, reward_type="dense",
    #                environment_loop_rate=10.0, action_cycle_time=0.8, seed=10, use_smoothing=True)

    # # discrete action space
    # env = gym.make('RX200ReacherDiscreteEnvSim-v0', gazebo_gui=False, delta_coeff=0.05, real_time=True,
    #                environment_loop_rate=10.0, action_cycle_time=0.8, seed=10, use_smoothing=False, reward_type="dense")

    # --- goal-conditioned environments
    # env = multiros_gym.make('RX200ReacherGoalEnvSim-v1', gazebo_gui=False, ee_action_type=False,
    #                delta_action=True, reward_type="sparse", real_time=True, environment_loop_rate=10.0,
    #                action_cycle_time=0.8, seed=10, use_smoothing=False)

    # env = gym.make('RX200ReacherGoalEnvSim-v2', gazebo_gui=False, delta_action=True,
    #                reward_type="sparse", real_time=True, environment_loop_rate=10.0, action_cycle_time=0.8, seed=10,
    #                use_smoothing=False)

    # discrete action space
    env = gym.make('RX200ReacherDiscreteGoalEnvSim-v0', gazebo_gui=True, delta_coeff=0.05, real_time=True,
                   environment_loop_rate=10.0, action_cycle_time=0.8, seed=10, use_smoothing=True,
                   reward_type="sparse")

    # Normalize action space
    # env = NormalizeActionWrapper(env)

    # Normalize observation space
    # env = NormalizeObservationWrapper(env)
    env = NormalizeObservationWrapper(env, normalize_goal_spaces=True)  # goal-conditioned environments

    # Set max steps
    # termination_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    # env = TimeLimitWrapper(env, max_steps=100, termination_action=termination_action)
    env = TimeLimitWrapper(env, max_steps=100)

    # reset the environment
    env.reset()

    # path to the package
    pkg_path = "reactorx200_ros_reacher"

    # # Default base environments - SAC
    # config_file_name = "sac.yaml"
    # save_path = "/models/sim/sac/"
    # log_path = "/logs/sim/sac/"

    # # create the model - SAC
    # model = SAC(env, save_path, log_path, model_pkg_path=pkg_path,
    #             config_file_pkg=pkg_path, config_filename=config_file_name)

    # # Default base environments - TD3
    # config_file_name = "td3.yaml"
    # save_path = "/models/sim/td3/"
    # log_path = "/logs/sim/td3/"
    #
    # # create the model - TD3
    # model = TD3(env, save_path, log_path, model_pkg_path=pkg_path,
    #             config_file_pkg=pkg_path, config_filename=config_file_name)

    # # Default base environments - discrete action space - DQN
    # config_file_name = "dqn.yaml"
    # save_path = "/models/sim/dqn/"
    # log_path = "/logs/sim/dqn/"
    #
    # # create the model - DQN
    # model = DQN(env, save_path, log_path, model_pkg_path=pkg_path,
    #             config_file_pkg=pkg_path, config_filename=config_file_name)

    # # Goal-conditioned environments - TD3+HER
    # config_file_name = "td3_goal.yaml"
    # save_path = "/models/sim/td3_goal/"
    # log_path = "/logs/sim/td3_goal/"
    #
    # # create the model
    # model = TD3_GOAL(env, save_path, log_path, model_pkg_path=pkg_path,
    #                  config_file_pkg=pkg_path, config_filename=config_file_name)

    # Goal-conditioned environments - DQN+HER
    config_file_name = "dqn_goal.yaml"
    save_path = "/models/sim/dqn_goal/"
    log_path = "/logs/sim/dqn_goal/"

    # create the model
    model = DQN_GOAL(env, save_path, log_path, model_pkg_path=pkg_path,
                     config_file_pkg=pkg_path, config_filename=config_file_name)

    # train the models
    model.train()
    model.save_model()
    model.close_env()

    sys.exit()
