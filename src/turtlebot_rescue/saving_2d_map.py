#!/usr/bin/env python

import rospy
import os
import json
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# File paths to save data
map_file = os.path.expanduser('~/ee106a-aen/Desktop/FinalProjectTurtle/src/turtlebot_rescue/saved_data/slam_map')
pose_file = os.path.expanduser('~/ee106a-aen/Desktop/FinalProjectTurtle/src/turtlebot_rescue/saved_data/robot_pose.json')

# Global variables for data
robot_pose = None

def odom_callback(data):
    global robot_pose
    # Extract position and orientation
    robot_pose = {
        'position': {
            'x': data.pose.pose.position.x,
            'y': data.pose.pose.position.y,
            'z': data.pose.pose.position.z
        },
        'orientation': {
            'x': data.pose.pose.orientation.x,
            'y': data.pose.pose.orientation.y,
            'z': data.pose.pose.orientation.z,
            'w': data.pose.pose.orientation.w
        }
    }

def save_data(map_path, pose_path):
    global robot_pose

    # Save pose
    if robot_pose:
        with open(pose_path, 'w') as pose_file:
            json.dump(robot_pose, pose_file)
        rospy.loginfo(f"Saved robot pose to {pose_path}")

    # Save map using ROS command
    os.system(f"rosrun map_server map_saver -f {map_path}")
    rospy.loginfo(f"Saved SLAM map to {map_path}.pgm and {map_path}.yaml")

def main():
    rospy.init_node('save_scout_data', anonymous=True)

    # Subscribe to odometry topic
    rospy.Subscriber('/odom', Odometry, odom_callback)

    rospy.loginfo("Drive the robot to map the environment. Press Ctrl+C when done.")
    rospy.spin()

    # Save data after exploration
    save_data(map_file, pose_file)

if __name__ == '__main__':
    main()