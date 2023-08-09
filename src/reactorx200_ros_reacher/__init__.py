#  we can register the environment with:

from gym.envs.registration import register

# ---------------------------- Simulation Environments  ----------------------------

# RX200 Reacher Multiros Default Environment
register(
    id='RX200ReacherEnvSim-v0',
    entry_point='reactorx200_ros_reacher.sim.task_envs.reactorx200_reacher_sim:RX200ReacherEnv',
    max_episode_steps=100,
)

# RX200 Reacher Multiros Goal Environment
register(
    id='RX200ReacherGoalEnvSim-v0',
    entry_point='reactorx200_ros_reacher.sim.task_envs.reactorx200_reacher_goal_sim:RX200ReacherGoalEnv',
    max_episode_steps=100,
)


# ---------------------------- Real Environments  ----------------------------

# RX200 Reacher ROS_RL Default Environment

register(
    id='RX200ReacherEnvReal-v0',
    entry_point='reactorx200_ros_reacher.real.task_envs.reactorx200_reacher_real:RX200ReacherEnv',
    max_episode_steps=100,
)

# RX200 Reacher ROS_RL Goal Environment

register(
    id='RX200ReacherGoalEnvReal-v0',
    entry_point='reactorx200_ros_reacher.real.task_envs.reactorx200_reacher_goal_real:RX200ReacherGoalEnv',
    max_episode_steps=100,
)
