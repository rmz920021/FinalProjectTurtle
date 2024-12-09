#!/usr/bin/env python3

import os
import rospy
import time
import shutil
from os.path import expanduser

# Define paths for saving data
home = expanduser("~")
db_source = os.path.join(home, ".ros", "rtabmap.db")
db_dest = os.path.join(home, "Desktop", "FinalProjectTurtle", "src", "turtlebot_rescue", "saved_data", "3d_map.db")
odom_file = os.path.join(home, "Desktop", "FinalProjectTurtle", "src", "turtlebot_rescue", "saved_data", "robot_odom.txt")
tf_file = os.path.join(home, "Desktop", "FinalProjectTurtle", "src", "turtlebot_rescue", "saved_data", "robot_tf.txt")

def save_3d_map_db():
    """Copies the RTAB-Map database file (3D map) to the specified location."""
    if os.path.exists(db_source):
        rospy.loginfo("Copying 3D map database...")
        shutil.copyfile(db_source, db_dest)
        rospy.loginfo(f"3D map database saved at {db_dest}")
    else:
        rospy.logwarn(f"No RTAB-Map database found at {db_source}. Make sure RTAB-Map is running and building a map.")

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
    rospy.loginfo("Waiting for ROS topics and RTAB-Map database to become available...")
    time.sleep(5)

    try:
        save_3d_map_db()
        save_robot_odometry()
        save_robot_tf()
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

    rospy.loginfo("All data saved successfully.")

if __name__ == "__main__":
    main()
