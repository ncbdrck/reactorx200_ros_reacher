#  we can register the environment with:

from gym.envs.registration import register

# ---------------------------- Simulation Environments  ----------------------------

# RX200 Reacher Multiros Default Environment
register(
    id='RX200ReacherEnvSim-v0',
    entry_point='reactorx200_ros_reacher.sim.task_envs.reactorx200_reacher_sim:RX200ReacherEnv',
    max_episode_steps=1000,
)

# RX200 Reacher Multiros Goal Environment
register(
    id='RX200ReacherGoalEnvSim-v0',
    entry_point='reactorx200_ros_reacher.sim.task_envs.reactorx200_reacher_goal_sim:RX200ReacherGoalEnv',
    max_episode_steps=1000,
)

# RX200 Reacher Multiros Default Environment v1
register(
    id='RX200ReacherEnvSim-v1',
    entry_point='reactorx200_ros_reacher.sim.task_envs.reactorx200_reacher_sim_v1:RX200ReacherEnv',
    max_episode_steps=1000,
)

# RX200 Reacher Multiros Goal Environment v1
register(
    id='RX200ReacherGoalEnvSim-v1',
    entry_point='reactorx200_ros_reacher.sim.task_envs.reactorx200_reacher_goal_sim_v1:RX200ReacherGoalEnv',
    max_episode_steps=1000,
)

# RX200 Reacher Multiros Default Environment v2
register(
    id='RX200ReacherEnvSim-v2',
    entry_point='reactorx200_ros_reacher.sim.task_envs.reactorx200_reacher_sim_v2:RX200ReacherEnv',
    max_episode_steps=1000,
)

# RX200 Reacher Multiros Goal Environment v2
register(
    id='RX200ReacherGoalEnvSim-v2',
    entry_point='reactorx200_ros_reacher.sim.task_envs.reactorx200_reacher_goal_sim_v2:RX200ReacherGoalEnv',
    max_episode_steps=1000,
)


# ---------------------------- Real Environments  ----------------------------

# RX200 Reacher ROS_RL Default Environment
register(
    id='RX200ReacherEnvReal-v0',
    entry_point='reactorx200_ros_reacher.real.task_envs.reactorx200_reacher_real:RX200ReacherEnv',
    max_episode_steps=1000,
)

# RX200 Reacher ROS_RL Goal Environment
register(
    id='RX200ReacherGoalEnvReal-v0',
    entry_point='reactorx200_ros_reacher.real.task_envs.reactorx200_reacher_goal_real:RX200ReacherGoalEnv',
    max_episode_steps=1000,
)

# RX200 Reacher ROS_RL Default Environment v1
register(
    id='RX200ReacherEnvReal-v1',
    entry_point='reactorx200_ros_reacher.real.task_envs.reactorx200_reacher_real_v1:RX200ReacherEnv',
    max_episode_steps=1000,
)

# RX200 Reacher ROS_RL Goal Environment v1
register(
    id='RX200ReacherGoalEnvReal-v1',
    entry_point='reactorx200_ros_reacher.real.task_envs.reactorx200_reacher_goal_real_v1:RX200ReacherGoalEnv',
    max_episode_steps=1000,
)

# RX200 Reacher ROS_RL Default Environment v2
register(
    id='RX200ReacherEnvReal-v2',
    entry_point='reactorx200_ros_reacher.real.task_envs.reactorx200_reacher_real_v2:RX200ReacherEnv',
    max_episode_steps=1000,
)

# RX200 Reacher ROS_RL Goal Environment v2
register(
    id='RX200ReacherGoalEnvReal-v2',
    entry_point='reactorx200_ros_reacher.real.task_envs.reactorx200_reacher_goal_real_v2:RX200ReacherGoalEnv',
    max_episode_steps=1000,
)
