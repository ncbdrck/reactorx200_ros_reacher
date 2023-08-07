# Simple Reach Task for ROS_RL and MultiROS

This repository contains experiments conducted to showcase the capabilities of the [ROS_RL](https://github.com/ncbdrck/ros_rl) and [MultiROS](https://github.com/ncbdrck/multiros) frameworks. 

Here we show how to train a simple reach task using the [Rx200](https://www.trossenrobotics.com/reactorx-200-robot-arm.aspx) robot.
 
This repo demonstrate the following features:
 1. Training the task directly in the real world (No need of a simulation environment - only [ROS_RL](https://github.com/ncbdrck/ros_rl) is required). 
 2. Training in simulation (using [MultiROS](https://github.com/ncbdrck/multiros) package) and deploying the trained model in the real world.
 3. Real-time training with simulation and real-world data for obtaining better generalization.

## Prerequisites

Before installing this package, make sure you have the following prerequisites:

### 1. ROS_RL

This ROS repo requires **ROS_RL** to train or evaluate the reach task in the real world. Please follow the instructions in the [ROS_RL repository](https://github.com/ncbdrck/ros_rl) to install ROS_RL.

### 2. MultiROS

To simulate the task in Gazebo, you need to install the **MultiROS** package. Please follow the instructions in the [MultiROS repository](https://github.com/ncbdrck/multiros) to install MultiROS.

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

### 4. Rx200 Robot description package

This package contains the URDF description of the Rx200 robot. It is a modified version of the original URDF and is necessary to execute this example. You can download it from [here](https://github.com/ncbdrck/rx200_supporting_materials) and follow the instructions to install it.
```shell
cd ~/catkin_ws/src
git clone https://github.com/ncbdrck/rx200_supporting_materials
cd ~/catkin_ws
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
## Issues

If you have installed the Rx200 robot package from the official documentation, you may face an error when running the scripts in this repository. This occurs because the official installation creates a new ROS workspace in your home directory and adds it to the bashrc file. As a result, each time you open a new terminal, the RX200 ROS workspace in your home directory is sourced instead of your catkin_ws directory. To resolve this, you have two options:

1. modify the line in the bashrc file that sources the RX200 ROS workspace. This will ensure that your catkin_ws directory is sourced instead when opening a new terminal.

2. Source the catkin_ws directory explicitly before running the scripts in this repository. This will override the sourcing of the RX200 ROS workspace and ensure that the correct workspace is used.

By taking one of these steps, you can avoid the error and ensure that the correct workspace is sourced when running the scripts in this repository.

To source the catkin_ws directory, run the following commands in a new terminal:
```shell
cd ~/catkin_ws/
source devel/setup.bash
rospack profile
```
If you encounter any issues while installing or running the scripts in the repo, please open an issue in this repository.


## Contact

For questions, suggestions, or collaborations, feel free to reach out to the project maintainer at [j.kapukotuwa@research.ait.ie](mailto:j.kapukotuwa@research.ait.ie).
