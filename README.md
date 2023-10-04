# Reach Task for ROS_RL and MultiROS

This repository contains experiments conducted to showcase the capabilities of the [ROS_RL](https://github.com/ncbdrck/ros_rl) and [MultiROS](https://github.com/ncbdrck/multiros) frameworks. 

Here we show how to train a simple reach task using the [Rx200](https://www.trossenrobotics.com/reactorx-200-robot-arm.aspx) robot.
 
This repo demonstrates the following features:
 1. Training the task directly in the real world (No need for a simulation environment - only [ROS_RL](https://github.com/ncbdrck/ros_rl) is required). 
 2. Training in simulation (using [MultiROS](https://github.com/ncbdrck/multiros) package) and deploying the trained model in the real world.
 3. Real-time training with simulation and real-world data for obtaining better generalisation.

## Prerequisites

Before installing this package, make sure you have the following prerequisites:

### 1. ROS_RL

This ROS repo requires **ROS_RL** to train or evaluate the reach task in the real world. Please follow the instructions in the [ROS_RL repository](https://github.com/ncbdrck/ros_rl) to install ROS_RL.

### 2. MultiROS

To simulate the task in Gazebo, you must install the **MultiROS** package. Please follow the instructions in the [MultiROS repository](https://github.com/ncbdrck/multiros) to install MultiROS.

### 3. Rx200 Robot Repository

You can download the official repository of the Rx200 robot from [here](https://github.com/Interbotix/interbotix_ros_manipulators) and follow the instructions to install it.

Furthermore, you can also follow the instructions in the [Rx200 Official Documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros1/software_setup.html) to install the related ros packages.

At the moment, these are the installation instructions for the Rx200 robot with ROS Noetic on Ubuntu 20.04:

```shell
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d noetic
```
**Note**: This will also install ROS Noetic (if not already installed) and create a new ROS workspace in your home directory. So source your workspace accordingly.

### 4. Rx200 Robot description package and other supporting packages

These packages contain the URDF description of the Rx200 robot. It is a modified version of the original URDF and is necessary to execute this example. 
```
cd ~/catkin_ws/src
git clone https://github.com/ncbdrck/reactorx200_description.git
git clone https://github.com/ncbdrck/common-sensors.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

### 5. SB3 ROS Support Package

This package contains the necessary scripts to train and evaluate the reach task using Stable Baselines3. You can download it from [here](https://github.com/ncbdrck/sb3_ros_support) and follow the instructions to install it.
```shell
# download the package
cd ~/catkin_ws/src
git clone https://github.com/ncbdrck/sb3_ros_support.git

# install the required Python packages by running
cd ~/catkin_ws/src/sb3_ros_support/
pip3 install -r requirements.txt

# build the ROS packages and source the environment:
cd ~/catkin_ws/
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

Please note that the instructions assume you are using Ubuntu 20.04 and ROS Noetic. If you are using a different operating system or ROS version, make sure to adapt the commands accordingly.

## Installation

Follow these steps to install this package:

1. Clone the repository:
    ```shell
    cd ~/catkin_ws/src
    git clone https://github.com/ncbdrck/reactorx200_ros_reacher
    ```

2. This package relies on several Python packages. You can install them by running the following command:

    ```shell
    # Install pip if you haven't already by running this command
    sudo apt-get install python3-pip

    # install the required Python packages by running
    cd ~/catkin_ws/src/reactorx200_ros_reacher/
    pip3 install -r requirements.txt
    ```
3. Build the ROS packages and source the environment:
    ```shell
   cd ~/catkin_ws/
   rosdep install --from-paths src --ignore-src -r -y
   catkin build
   source devel/setup.bash
   rospack profile
    ```
   
## Usage

This repo contains both simulated and real-world environments. Out of these environments 
- All the environments **only** supports **continuous** action spaces and **continuous** observation spaces 
- `v0` is now deprecated
- `v1` - Supports both sequential and asynchronous learning and uses **Moveit** to move the robot
- `v1` - Also supports controlling the robots with joint positions (5 elements) or by giving the end-effector 3D position (3 elements) as actions. (Must be set when initialising the environment. **Default** is Joint positions)
- `v2` - Supports sequential and asynchronous learning and uses **ROS Controllers** (Joint positions) to move the robot. Doesn't support end-effector 3D position.

### 1. Available Environments

**Simulation based on Gazebo:**
- **Base Envs:**
   - `RX200ReacherEnvSim-v0` (deprecated)
   - `RX200ReacherEnvSim-v1` (both sequential and asynchronous - MoveIt)
   - `RX200ReacherEnvSim-v2` (both sequential and asynchronous - ROS control)


- **Goal-Conditioned Envs:**
   - `RX200ReacherGoalEnvSim-v0` (deprecated)
   - `RX200ReacherGoalEnvSim-v1` (both sequential and asynchronous — MoveIt)
   - `RX200ReacherGoalEnvSim-v2` (both sequential and asynchronous - ROS control)

The env parameters are explained in the following table:

**Common** parameters for all simulated environments and their **default** values:

| Parameter                     | Description                                                           | Base | Goal-Conditioned |
|-------------------------------|-----------------------------------------------------------------------|---------|------------------|
| launch_gazebo (bool)          | Whether to launch Gazebo or not.                                      | True    | True             |
| new_roscore (bool)            | Whether to launch a new roscore or not.                               | True    | True             |
| roscore_port (str)            | Port of the roscore to be launched. If None, a random port is chosen. | None    | None             |
| gazebo_paused (bool)          | Whether to start Gazebo in a paused state or not.                     | False   | False            |
| gazebo_gui (bool)             | Whether to start Gazebo with the GUI or not.                          | False   | False            |
| seed (int)                    | Seed for the environment.                                             | None    | None             |
| reward_type (str)             | Type of reward function to use.                                       | 'dense' | 'sparse'         |
| delta_action (bool)           | Whether to use delta actions or not.                                  | False   | True             |
| delta_coeff (float)           | Coefficient for the delta action.                                     | 0.05    | 0.05             |
| real_time (bool)              | Whether to run the simulation in real-time or not. Don't pause Gazebo and use asynchronous learning mode if 'True'                     | False   | True             |
| environment_loop_rate (float) | Rate at which the environment should run in Hz. (only for real-time)  | None    | None             |
| action_cycle_time (float)     | Time to wait between two consecutive actions.                         | 0.0     | 0.0              |
| use_smoothing (bool)          | Whether to use action smoothing or not. 'delta_action' should be 'True'. Use 'delta_coeff' if 'action_cycle_time' is zero                                 | False   | False            |



**Real-World:**
- **Base Envs:**
   - `RX200ReacherEnvReal-v0` (deprecated)
   - `RX200ReacherEnvReal-v1` (both sequential and asynchronous - MoveIt)
   - `RX200ReacherEnvReal-v2` (both sequential and asynchronous - ROS control)


- **Goal-Conditioned Envs:**
   - `RX200ReacherGoalEnvReal-v0` (deprecated)
   - `RX200ReacherGoalEnvReal-v1` (both sequential and asynchronous — MoveIt)
   - `RX200ReacherGoalEnvReal-v2` (both sequential and asynchronous - ROS control)

**Common** parameters for all real-world environments and their default values:

| Parameter                     | Description                                                           | Base | Goal-Conditioned |
|-------------------------------|-----------------------------------------------------------------------|---------|------------------|
| new_roscore (bool)            | Whether to launch a new roscore or not.                               | True    | True             |
| roscore_port (str)            | Port of the roscore to be launched. If None, a random port is chosen. | None    | None             |
| seed (int)                    | Seed for the environment.                                             | None    | None             |
| reward_type (str)             | Type of reward function to use.                                       | 'dense' | 'sparse'         |
| delta_action (bool)           | Whether to use delta actions or not.                                  | False   | True             |
| delta_coeff (float)           | Coefficient for the delta action.                                     | 0.05    | 0.05             |
| real_time (bool)              | Whether to run in asynchronous learning mode or not.                  | False   | True             |
| environment_loop_rate (float) | Rate at which the environment should run in Hz. (only for real-time)  | None    | None             |
| action_cycle_time (float)     | Time to wait between two consecutive actions.                         | 0.0     | 0.0              |
| use_smoothing (bool)          | Whether to use action smoothing or not. 'delta_action' should be 'True'. Use 'delta_coeff' if 'action_cycle_time' is zero                                 | False   | False            |


**Unique parameters for both environment types of both simulated and real-world are explained in the following table:**

| Parameter             | Description                                           | v1 | v2 |
|-----------------------|-------------------------------------------------------|----|----|
| ee_action_type (bool) | Whether to use end-effector actions or joint actions. | ✔️ | ❌  |


**Observation Space:**

| Observation                    | Description                                                | Elements             |
|--------------------------------|------------------------------------------------------------|----------------------|
| End effector position          | Position of the end effector in the world frame            | 3                    |
| Vector to the goal             | Normalized linear distance between the EE pos and the Goal | 3                    |
| Euclidian distance to the goal | Euclidian distance between the EE pos and the Goal         | 1                    |
| Current joint angles           | Current joint angles of the robot                          | 8                    |
| Previous action                | Previous action taken by the agent                         | 5 or 3 (joint or ee) |
| Joint velocities               | Current joint velocities of the robot                      | 8                    |


**Action Space:**

| Action          | Description                                 | v1 | v2 | Elements |
|-----------------|---------------------------------------------|----|----|----------|
| EE Position     | 3D end-effector position in the world frame | ✔️ | ❌  | 3        |
| Joints position | joint posiions of the robot                | ✔️ | ✔️ | 5        |

### 2. Training in the real world or simulation
 
- The first step is to check the `train_sim.py` or `train_real.py` files in the scripts folder
and modify the parameters accordingly.
- The RL model parameters are in the `config` folder in the project repo.
- The task configuration is also found in the `config` folder inside the project repo. (`reach_task_config_v1.yaml`)

**Simulation**:
```shell
rosrun reactorx200_ros_reacher train_sim.py
```

**Real-World**:
```shell
rosrun reactorx200_ros_reacher train_real.py
```

or if you prefer to use your own RL algorithm, you can get started by using the following structure:

```python
#!/bin/python3
import rospy
import gym

import reactorx200_ros_reacher
from multiros.core import multiros_gym

# wrappers
from multiros.wrappers.normalize_action_wrapper import NormalizeActionWrapper
from multiros.wrappers.normalize_obs_wrapper import NormalizeObservationWrapper
from multiros.wrappers.time_limit_wrapper import TimeLimitWrapper

if __name__ == '__main__':
    
    env = gym.make('RX200ReacherEnvSim-v2', gazebo_gui=False, delta_action=True, real_time=True, reward_type="dense",
                   environment_loop_rate=50.0, action_cycle_time=0.2, seed=0)
    
    # # or 
    # env = multiros_gym.make('RX200ReacherEnvSim-v2', gazebo_gui=False, delta_action=True, real_time=True, reward_type="dense",
    #                environment_loop_rate=50.0, action_cycle_time=0.2, seed=0)
    
    env = NormalizeActionWrapper(env)
    env = NormalizeObservationWrapper(env)
    env = TimeLimitWrapper(env, max_steps=100)
    
    # your RL algorithm
    # ...
    
    env.close()
```

### 3. Evaluating the trained model

- The first step is to check the `validate_sim.py` or `evalidate_real.py` files in the scripts folder
- The RL model parameters are in the `config` folder in the project repo.

**Simulation**:
```shell
rosrun reactorx200_ros_reacher validate_sim.py
```

**Real-World**:
```shell
rosrun reactorx200_ros_reacher validate_real.py
```

or if you prefer to use your own RL algorithm, you can get started by using the following structure:

```python
#!/bin/python3
import rospy
import gym
import sys

import reactorx200_ros_reacher
from multiros.core import multiros_gym

# wrappers
from multiros.wrappers.normalize_action_wrapper import NormalizeActionWrapper
from multiros.wrappers.normalize_obs_wrapper import NormalizeObservationWrapper
from multiros.wrappers.time_limit_wrapper import TimeLimitWrapper


if __name__ == '__main__':
    
    env = gym.make('RX200ReacherEnvSim-v2', gazebo_gui=False, delta_action=True, real_time=True, reward_type="dense",
                   environment_loop_rate=50.0, action_cycle_time=0.2, seed=0)
    
    # # or 
    # env = multiros_gym.make('RX200ReacherEnvSim-v2', gazebo_gui=False, delta_action=True, real_time=True, reward_type="dense",
    #                environment_loop_rate=50.0, action_cycle_time=0.2, seed=0)
    
    env = NormalizeActionWrapper(env)
    env = NormalizeObservationWrapper(env)
    env = TimeLimitWrapper(env, max_steps=100)
    
    # load your trained model
    # ...
    
    # evaluate the model
    obs = env.reset()
    episodes = 1000
    epi_count = 0
    while epi_count < episodes:
        action, _states = model.predict(observation=obs)
        obs, _, dones, info = env.step(action)
        if dones:
            epi_count += 1
            rospy.logwarn("Episode: " + str(epi_count))
            obs = env.reset()

    env.close()
    sys.exit()

```


## Issues

If you have installed the Rx200 robot package from the official documentation, you may face an error when running the scripts in this repository. This occurs because the official installation creates a new ROS workspace in your home directory and 
adds it to the `~/.bashrc` file. As a result, each time you open a new terminal, the RX200 ROS workspace in your home directory is sourced instead of your `catkin_ws` directory. To resolve this, you have two options:

1. modify the line in the `~/.bashrc` file that sources the RX200 ROS workspace. This will ensure that your `catkin_ws` directory is sourced instead when opening a new terminal.

2. Source the catkin_ws directory explicitly before running the scripts in this repository. This will override the sourcing of the RX200 ROS workspace and ensure that the correct workspace is used.

By taking one of these steps, you can avoid the error and ensure that the correct workspace is sourced when running the scripts in this repository.

To source the `catkin_ws` directory, run the following commands in a new terminal:
```shell
cd ~/catkin_ws/
source devel/setup.bash
rospack profile
```
If you encounter any issues while installing or running the scripts in the repo, please open an issue in this repository.


## Contact

For questions, suggestions, or collaborations, feel free to reach out to the project maintainer at [j.kapukotuwa@research.ait.ie](mailto:j.kapukotuwa@research.ait.ie).
