# Example Reach Task for ROS_RL

This repository contains an example reach task for the [ROS_RL](https://github.com/ncbdrck/ros_rl) framework. 
 
This framework provides the following features:
 1. Training the task directly in the real world (No need of a simulation environment)
 2. Training in simulation (using [MultiROS](https://github.com/ncbdrck/multiros) package) and deploying the trained model in the real world
 3. Real-time training with simulation and real-world data for obtaining better generalization

## Prerequisites

Before installing this package, make sure you have the following prerequisites:

### 1. ROS_RL

This ROS repo requires ROS_RL to be installed. Please follow the instructions in the [ROS_RL repository](https://github.com/ncbdrck/ros_rl) to install ROS_RL.

### 2. MultiROS

To simulate the task, you need to install the [MultiROS](https://github.com/ncbdrck/multiros) package. Please follow the instructions in the package to install it.

### 3. Rx200 Robot repository

You can download the official repository of the Rx200 robot from [here](https://github.com/Interbotix/interbotix_ros_manipulators) and follow the instructions to install it.

Furthermore, you can also follow the instructions in the [Rx200 official Documentation](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros1/software_setup.html) to install the related ros packages.

At the moment, these are the installation instructions for the Rx200 robot with ROS Noetic on Ubuntu 20.04:

```shell
sudo apt install curl
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
./xsarm_amd64_install.sh -d noetic
```
**Note**: Unfortunately, this will create a new catkin workspace in your home directory.

### 4. Rx200 Robot description package

This package contains the URDF description of the Rx200 robot. You can download it from [here]() and follow the instructions to install it.
```shell
cd ~/catkin_ws/src
git clone https://github.com/ncbdrck/reactorx200_description
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin build
source devel/setup.bash
```

Please note that the instructions assume you are using Ubuntu 20.04 and ROS Noetic. If you are using a different operating system or ROS version, make sure to adapt the commands accordingly.

## Installation

Follow these steps to install the example reach task

1. Clone the repository:
    ```shell
    cd ~/catkin_ws/src
    git clone https://github.com/ncbdrck/reactorx200_ros_reacher
    ```

2. This package relies on several Python packages. You can install them by running the following command:

    ```shell
    # Install pip if you haven't already by running this command
    sudo apt-get install python3-pip

    # install the required Python packages for ROS_RL by running
    cd ~/catkin_ws/src/reactorx200_description/
    pip3 install -r requirements.txt
    ```
3. Build the ROS packages and source the environment:
    ```shell
   cd ~/catkin_ws/
   rosdep install --from-paths src --ignore-src -r -y
   catkin build
   source devel/setup.bash
    ```

## Contact

For questions, suggestions, or collaborations, feel free to reach out to the project maintainer at [j.kapukotuwa@research.ait.ie](mailto:j.kapukotuwa@research.ait.ie).
