#!/usr/bin/env python3

import os
import rospy
import time
import shutil
from os.path import expanduser
from geometry_msgs.msg import PoseWithCovarianceStamped

# Define paths for saving data
home = expanduser("~")
db_source = os.path.join(home, ".ros", "rtabmap.db")
db_dest = os.path.join(home, "Desktop", "FinalProjectTurtle", "src", "turtlebot_rescue", "saved_data", "3d_map.db")
odom_file = os.path.join(home, "Desktop", "FinalProjectTurtle", "src", "turtlebot_rescue", "saved_data", "robot_odom.txt")
tf_file = os.path.join(home, "Desktop", "FinalProjectTurtle", "src", "turtlebot_rescue", "saved_data", "robot_tf.txt")
pose_file = os.path.join(home, "Desktop", "FinalProjectTurtle", "src", "turtlebot_rescue", "saved_data", "robot_odom.txt")

# Output YAML map name (without extension, map_saver will append .yaml and .pgm)
yaml_map_name = "3d_map"
yaml_map_base = os.path.join(home, "Desktop", "FinalProjectTurtle", "src", "turtlebot_rescue", "saved_data", yaml_map_name)

def pose_callback(msg):
    """Callback to save the current pose to a YAML file."""
    rospy.loginfo("Saving current pose to file...")
    with open(pose_file, 'w') as f:
        f.write("header:\n")
        f.write(f"  seq: {msg.header.seq}\n")
        f.write(f"  stamp:\n")
        f.write(f"    secs: {msg.header.stamp.secs}\n")
        f.write(f"    nsecs: {msg.header.stamp.nsecs}\n")
        f.write(f"  frame_id: \"{msg.header.frame_id}\"\n")
        f.write(f"pose:\n")
        f.write(f"  pose:\n")
        f.write(f"    position:\n")
        f.write(f"      x: {msg.pose.pose.position.x}\n")
        f.write(f"      y: {msg.pose.pose.position.y}\n")
        f.write(f"      z: {msg.pose.pose.position.z}\n")
        f.write(f"    orientation:\n")
        f.write(f"      x: {msg.pose.pose.orientation.x}\n")
        f.write(f"      y: {msg.pose.pose.orientation.y}\n")
        f.write(f"      z: {msg.pose.pose.orientation.z}\n")
        f.write(f"      w: {msg.pose.pose.orientation.w}\n")
    rospy.loginfo(f"Pose saved at {pose_file}")

def save_pose():
    # rospy.init_node('save_current_pose', anonymous=True)
    
    rospy.loginfo("Waiting for /rtabmap/localization_pose topic...")
    rospy.wait_for_message("/rtabmap/localization_pose", PoseWithCovarianceStamped, timeout=30)
    
    rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, pose_callback)
    
    rospy.loginfo("Subscribed to /rtabmap/localization_pose. Waiting to save pose...")
    
    # Wait for a short duration to ensure the callback is triggered
    time.sleep(2)
    
    rospy.loginfo("Pose saving node finished.")

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

def save_yaml_map():
    """Saves the 2D occupancy grid (YAML + PGM) using map_saver."""
    rospy.loginfo("Saving 2D occupancy grid map...")

    # Adjust the map topic if necessary. 
    # Common RTAB-Map occupancy grid topic: /rtabmap/grid_map
    # If your occupancy grid is published elsewhere, change map:= argument.
    # This will produce my_map.yaml and my_map.pgm in the same directory.
    cmd = f"rosrun map_server map_saver map:=/rtabmap/grid_map -f {yaml_map_base}"
    ret = os.system(cmd)
    if ret == 0:
        rospy.loginfo(f"2D map saved at {yaml_map_base}.yaml and {yaml_map_base}.pgm")
    else:
        rospy.logwarn("Failed to save the map. Ensure /rtabmap/grid_map is published.")

def main():
    rospy.init_node('save_robot_data', anonymous=True)

    # Allow time for the ROS environment to initialize
    rospy.loginfo("Waiting for ROS topics and RTAB-Map database to become available...")
    time.sleep(5)

    try:
        save_3d_map_db()
        save_pose()
        # save_robot_odometry()
        # save_robot_tf()
        save_yaml_map()
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

    rospy.loginfo("All data saved successfully.")

if __name__ == "__main__":
    main()
