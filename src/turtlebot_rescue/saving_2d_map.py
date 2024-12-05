#!/usr/bin/env python3

import os
import rospy
import time

# Define paths for saving data
map_file = "~/Desktop/FinalProjectTurtle/src/turtlebot_rescue/saved_data/slam_map"
odom_file = "~/Desktop/FinalProjectTurtle/src/turtlebot_rescue/saved_data/robot_odom.txt"
tf_file = "~/Desktop/FinalProjectTurtle/src/turtlebot_rescue/saved_data/robot_tf.txt"

def save_slam_map():
    """Saves the SLAM map using the map_saver command."""
    rospy.loginfo("Saving SLAM map...")
    os.system(f"rosrun map_server map_saver -f {map_file}")
    rospy.loginfo(f"SLAM map saved at {map_file}.pgm and {map_file}.yaml")

def save_robot_odometry():
    """Saves the robot's odometry to a file."""
    rospy.loginfo("Saving robot odometry...")
    os.system(f"rostopic echo -n 1 /odom > {odom_file}")
    rospy.loginfo(f"Robot odometry saved at {odom_file}")

def save_robot_tf():
    """Saves the robot's transformation data to a file."""
    rospy.loginfo("Saving robot transformation data...")
    os.system(f"rostopic echo -n 1 /tf > {tf_file}")
    rospy.loginfo(f"Robot transformation data saved at {tf_file}")

def main():
    rospy.init_node('save_robot_data', anonymous=True)

    # Allow time for the ROS environment to initialize
    rospy.loginfo("Waiting for ROS topics to become active...")
    time.sleep(5)

    try:
        save_slam_map()
        save_robot_odometry()
        save_robot_tf()
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

    rospy.loginfo("All data saved successfully.")

if __name__ == "__main__":
    main()