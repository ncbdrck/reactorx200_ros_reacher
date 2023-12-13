#!/bin/python3

# ROS packages required
import rospy

# gym
import gym

# Multiros and realros packages
from realros.core import realros_gym
from realros.utils import ros_common as realros_common
from multiros.core import multiros_gym
from multiros.utils import ros_common as multiros_common

# Environments are registered inside the main __init__.py of the reactorx200_ros_reacher package
import reactorx200_ros_reacher

# Models
from sb3_ros_support.sac import SAC
from sb3_ros_support.td3_goal import TD3_GOAL

# wrappers
from multiros.wrappers.normalize_action_wrapper import NormalizeActionWrapper
from multiros.wrappers.normalize_obs_wrapper import NormalizeObservationWrapper
from multiros.wrappers.time_limit_wrapper import TimeLimitWrapper

# Other packages
import sys
import numpy as np
import wandb
import matplotlib.pyplot as plt

if __name__ == '__main__':
    # Kill all processes related to previous runs
    # realros_common.kill_all_ros_processes()
    # multiros_common.kill_all_ros_and_gazebo()

    # --- normal environments
    env = multiros_gym.make('RX200ReacherEnvSim-v0', gazebo_gui=False, ee_action_type=False,
                            delta_action=False)  # sim
    # env = realros_gym.make('RX200ReacherEnvReal-v0', ee_action_type=False, delta_action=False)  # real

    # --- goal-conditioned environments
    # env = multiros_gym.make('RX200ReacherGoalEnvSim-v0', gazebo_gui=False, ee_action_type=False,
    #                delta_action=False, reward_type="sparse")  # sim
    # env = realros_gym.make('RX200ReacherGoalEnvReal-v0', ee_action_type=False, delta_action=False,
    #                       reward_type="sparse")  # real

    # initialize wandb
    wandb.init(project="reach_ijrr")

    # Normalize action space
    env = NormalizeActionWrapper(env)

    # Normalize observation space
    env = NormalizeObservationWrapper(env)
    # env = NormalizeObservationWrapper(env, normalize_goal_spaces=True)  # goal-conditioned environments

    # Set max steps
    env = TimeLimitWrapper(env, max_steps=100)

    # reset the environment
    env.reset()

    # path to the package
    pkg_path = "reactorx200_ros_reacher"

    # Default base environments (both sim and real)- SAC
    load_path = "/models/sim/sac/"
    # load_path = "/models/real/sac/"

    model = SAC.load_trained_model(load_path + "trained_model_name_without_.zip",
                                   model_pkg_path=pkg_path)

    # Goal-conditioned environments (both sim and real)- - TD3+HER
    # load_path = "/models/sim/td3_goal/"
    # load_path = "/models/real/td3_goal/"
    #
    # model = TD3_GOAL.load_trained_model(load_path + "trained_model_name_without_.zip",
    #                                     model_pkg_path=pkg_path)

    obs = env.reset()
    episodes = 100
    epi_count = 0
    success_count = 0

    # Lists to store goal positions and achievement status
    goal_positions = []
    achieved_list = []

    while epi_count < episodes:
        action, _states = model.predict(observation=obs, deterministic=True)
        obs, _, dones, info = env.step(action)
        if dones:
            epi_count += 1
            rospy.logwarn("Episode: " + str(epi_count))

            # Extract goal position and achievement from observations
            goal = obs[3:6]
            achieved = info["is_success"]

            # Store goal position and achievement status
            goal_positions.append(goal)
            achieved_list.append(achieved)

            # Update success count
            if achieved:
                success_count += 1

            obs = env.reset()

    # Calculate success rate
    success_rate = success_count / episodes
    print("sucess_rate", success_rate)

    # Convert lists to NumPy arrays
    goal_positions = np.array(goal_positions)
    achieved_list = np.array(achieved_list)

    # Plot the 3D scatter plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set colors for achieved and not achieved goals
    colors = np.where(achieved_list, 'green', 'red')

    # Plot the goals
    ax.scatter(goal_positions[:, 0], goal_positions[:, 1], goal_positions[:, 2], c=colors)

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # ax.set_title('Goal Achievement')

    # Adjust the tick frequency for the axis labels
    ax.xaxis.set_major_locator(plt.MaxNLocator(5))
    ax.yaxis.set_major_locator(plt.MaxNLocator(5))
    ax.zaxis.set_major_locator(plt.MaxNLocator(5))

    # Add success rate annotation
    # ax.text2D(0.05, 0.95, f"Success Rate: {success_rate * 100:.2f}%", transform=ax.transAxes)

    # Add legend
    # ax.legend(['Achieved', 'Not Achieved'])

    # Log the scatter plot to wandb
    wandb.log({"scatter_plot": wandb.Image(fig)})

    # Show the plot
    plt.show()

    env.close()
    sys.exit()

