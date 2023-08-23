#!/bin/python3
import numpy as np

import rospy
import rostopic
from std_msgs.msg import Float32MultiArray, Bool

from multiros.utils.moveit_multiros import MoveitMultiros
from multiros.utils import ros_common

import argparse
"""
Script to create a ROS node that publishes observations and executes actions at a given rate.

This is simply for testing purposes, to be able to test the environment without the need of a learning algorithm.
"""


class Robot_Loop:
    def __init__(self, observation_rate=10, action_rate=10, ros_port=None, gazebo_port=None, ee_action_type=False):

        # Change the ros and gazebo ports if needed
        if ros_port is not None:
            ros_common.change_ros_gazebo_master(ros_port=ros_port, gazebo_port=gazebo_port)

        # Initialize the node
        rospy.init_node('obs_and_actions_loop')

        rospy.loginfo("Start Init Robot_Loop")

        # parameters
        self.ee_action_type = ee_action_type

        # Launch the MoveIt! interface
        ros_common.ros_launch_launcher(pkg_name="interbotix_xsarm_moveit_interface",
                                       launch_file_name="xsarm_moveit_interface.launch",
                                       args=["robot_model:=rx200", "dof:=5", "use_python_interface:=true",
                                             "use_moveit_rviz:=false"])

        # check if moveit is ready
        self.check_moveit_ready()

        # Create a MoveitMultiros object
        self.move_RX200_object = MoveitMultiros(arm_name='interbotix_arm',
                                                gripper_name='interbotix_gripper',
                                                robot_description="rx200/robot_description",
                                                ns="rx200", pause_gazebo=False)

        # Create a publisher for the observations
        self.observation_pub = rospy.Publisher('/observations', Float32MultiArray, queue_size=10)

        # Create a subscriber for the actions
        self.action_sub = rospy.Subscriber('/actions', Float32MultiArray, self.action_callback)

        # Create a publisher for the actions
        self.action_status_pub = rospy.Publisher('/actions_status', Bool, queue_size=10)

        # Create a subscriber for the check_goal
        self.check_goal_sub = rospy.Subscriber('/check_goal', Float32MultiArray, self.check_goal_callback)

        # Create a publisher for the check_goal
        self.check_goal_pub = rospy.Publisher('/check_goal_status', Bool, queue_size=10)

        # Initialize the current action
        self.current_action = None

        # Create timers for publishing observations and executing actions
        rospy.Timer(rospy.Duration(1.0 / observation_rate), self.publish_observation)
        rospy.Timer(rospy.Duration(1.0 / action_rate), self.execute_action)

        rospy.loginfo("End Init Robot_Loop")

    def check_moveit_ready(self):
        """
        Function to check if moveit services are running
        """
        rospy.wait_for_service("/rx200/move_group/trajectory_execution/set_parameters")
        rospy.logdebug(rostopic.get_topic_type("/rx200/planning_scene", blocking=True))
        rospy.logdebug(rostopic.get_topic_type("/rx200/move_group/status", blocking=True))

        rospy.loginfo("Moveit is ready!")

        return True

    def check_goal_callback(self, msg):

        # Check if the goal can be reached
        goal = msg.data
        result = self.move_RX200_object.check_goal(goal)

        # Create the outcome
        outcome = Bool()
        outcome.data = result

        # Publish the outcome
        self.check_goal_pub.publish(outcome)

    def action_callback(self, msg):
        # Update the current action when a new action is received
        self.current_action = msg.data
        if self.ee_action_type:
            result = self.move_RX200_object.set_trajectory_ee(position=self.current_action, async_move=True)

            # Publish the action status
            action_status = Bool()
            action_status.data = result

            self.action_status_pub.publish(action_status)
        else:
            result = self.move_RX200_object.set_trajectory_joints(q_positions=self.current_action, async_move=True)

            # Publish the action status
            action_status = Bool()
            action_status.data = result

            self.action_status_pub.publish(action_status)

    def publish_observation(self, event):
        # Get the current observation
        observation = self.get_observation()

        # Publish the observation
        self.observation_pub.publish(observation)

    def get_observation(self):
        # Get the current observation (e.g. from sensors)
        observation = Float32MultiArray()

        ee_pos_tmp = self.move_RX200_object.get_robot_pose()
        ee_pos = np.array([ee_pos_tmp.pose.position.x, ee_pos_tmp.pose.position.y, ee_pos_tmp.pose.position.z])
        # ee_rpy = self.move_RX200_object.get_robot_rpy()
        current_joints = self.move_RX200_object.get_joint_angles_robot_arm()

        # concatenate the observation
        observation.data = np.concatenate((ee_pos, current_joints), axis=None)

        return observation

    def execute_action(self, event):
        # Execute the current action (if any)
        if self.current_action is not None:

            if self.ee_action_type:
                result = self.move_RX200_object.set_trajectory_ee(position=self.current_action, async_move=True)

                # Publish the action status
                action_status = Bool()
                action_status.data = result

                self.action_status_pub.publish(action_status)
            else:
                result = self.move_RX200_object.set_trajectory_joints(q_positions=self.current_action, async_move=True)

                # Publish the action status
                action_status = Bool()
                action_status.data = result

                self.action_status_pub.publish(action_status)


if __name__ == '__main__':

    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--observation_rate', type=int, default=10)
    parser.add_argument('--action_rate', type=int, default=10)
    parser.add_argument('--ros_port', type=str, default=None)
    parser.add_argument('--gazebo_port', type=str, default=None)
    parser.add_argument('--ee_action_type', type=bool, default=False)
    args = parser.parse_args()

    agent = Robot_Loop(observation_rate=args.observation_rate, action_rate=args.action_rate, ros_port=args.ros_port,
                       gazebo_port=args.gazebo_port, ee_action_type=args.ee_action_type)

    rospy.spin()
