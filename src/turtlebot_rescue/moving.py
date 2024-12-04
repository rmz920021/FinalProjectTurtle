#!/usr/bin/env python

import rospy
import os
import json
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class NavigateToLocation:
    def __init__(self):
        rospy.init_node('navigate_to_location', anonymous=True)

        # Action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # Load saved data
        self.data_path = os.path.expanduser("~/catkin_ws/src/turtlebot_rescue/saved_data")
        self.odometry = self.load_data('odometry.json')

    def load_data(self, filename):
        try:
            with open(os.path.join(self.data_path, filename), 'r') as file:
                return json.load(file)
        except FileNotFoundError:
            rospy.logerr(f"{filename} not found in {self.data_path}")
            return None

    def send_goal(self):
        if not self.odometry:
            rospy.logerr("No odometry data available to navigate!")
            return

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the target position and orientation
        goal.target_pose.pose.position.x = self.odometry['position']['x']
        goal.target_pose.pose.position.y = self.odometry['position']['y']
        goal.target_pose.pose.position.z = self.odometry['position']['z']
        goal.target_pose.pose.orientation.x = self.odometry['orientation']['x']
        goal.target_pose.pose.orientation.y = self.odometry['orientation']['y']
        goal.target_pose.pose.orientation.z = self.odometry['orientation']['z']
        goal.target_pose.pose.orientation.w = self.odometry['orientation']['w']

        rospy.loginfo(f"Navigating to position: {goal.target_pose.pose.position}")
        rospy.loginfo(f"With orientation: {goal.target_pose.pose.orientation}")

        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("Goal reached!")

if __name__ == '__main__':
    navigator = NavigateToLocation()
    navigator.send_goal()