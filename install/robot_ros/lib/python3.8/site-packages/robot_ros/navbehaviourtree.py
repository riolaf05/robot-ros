#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import BehaviorTreeStatus
from tree_attribute_parser import TreeAttributeParser
from launch_ros.substitutions import FindPackageShare
from action_msgs.msg import GoalStatus
from typing import List
from functools import partial

class NavBehaviorTree(Node):

    def __init__(self):
        super().__init__('nav_robot_ros')

        # Parse the behavior tree attributes
        package_name='robot_ros'
        pkg_share = FindPackageShare(package=package_name).find(package_name)
        self.tree = TreeAttributeParser(os.path.join(pkg_share, 'robot_ros', 'navigation_tree.xml'))
        self.current_goal_index = 0

        # Create a publisher to send goal poses
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        # Create an action client to navigate to the goal pose
        self.action_client = self.create_client(NavigateToPose, 'navigate_to_pose')
        self.action_client.wait_for_server()

        # Subscribe to the behavior tree status topic
        self.bt_status_sub = self.create_subscription(BehaviorTreeStatus, 'bt_status', self.bt_status_callback, 10)

        self.get_logger().info('Navigation behavior tree node initialized')

    def bt_status_callback(self, msg: BehaviorTreeStatus):
        # Check if the action client is active
        if self.action_client.is_active():
            # Check the status of the current action goal
            if msg.current_status.status == GoalStatus.STATUS_SUCCEEDED:
                self.current_goal_index += 1
                if self.current_goal_index < len(self.tree.goals):
                    # Publish the next goal pose
                    goal = self.tree.goals[self.current_goal_index]
                    goal_pose = PoseStamped()
                    goal_pose.pose.position.x = goal['position'][0]
                    goal_pose.pose.position.y = goal['position'][1]
                    goal_pose.pose.orientation.z = goal['orientation'][2]
                    goal_pose.header.frame_id = "base_link" #TODO or "map" ?
                    self.goal_pub.publish(goal_pose)

                    # Send the next goal pose to the action client
                    goal_msg = NavigateToPose.Goal()
                    goal_msg.pose = goal_pose.pose
                    self.action_client.send_goal_async(goal_msg)

        # If the action client is not active, send the first goal pose to the action client
        else:
            goal = self.tree.goals[self.current_goal_index]
            goal_pose = PoseStamped()
            goal_pose.pose.position.x = goal['position'][0]
            goal_pose.pose.position.y = goal['position'][1]
            goal_pose.pose.orientation.z = goal['orientation'][2]
            goal_pose.header.frame_id = "map"
            self.goal_pub.publish(goal_pose)

            # Send the first goal pose to the action client
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose.pose
            self.action_client.send_goal_async(goal_msg)

    def shutdown(self):
        # Cancel the current action goal and shutdown the node
        self.action_client.cancel_all_goals()
        super().shutdown()

def main(args=None):
    rclpy.init(args=args)
    nav_robot_ros = NavBehaviorTree()
    rclpy.spin(nav_robot_ros)
    nav_robot_ros.shutdown()
    rclpy.shutdown()
