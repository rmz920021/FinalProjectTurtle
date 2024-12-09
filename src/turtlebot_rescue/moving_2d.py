#!/usr/bin/env python3

import rospy
import json
import os
import yaml
import actionlib
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import subprocess

# File paths for saved data
map_file = os.path.expanduser("~/Desktop/FinalProjectTurtle/src/turtlebot_rescue/saved_data/slam_map.yaml")
odom_file = os.path.expanduser("~/Desktop/FinalProjectTurtle/src/turtlebot_rescue/saved_data/robot_odom.txt")
# subprocess.Popen(['roslaunch', "turtlebot3_navigation", "turtlebot3_navigation.launch","map_file:={map_file}"])
# os.system(f"roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:={map_file}")
def load_saved_pose(file_path):
    """Loads the saved robot pose from a file."""
    rospy.loginfo(f"Loading saved robot pose from {file_path}")
    try:
        with open(file_path, 'r') as file:
            yaml_docs = list(yaml.safe_load_all(file))
            rospy.loginfo(f"{yaml_docs}")
            yaml_data = yaml_docs[0]
            position = yaml_data['pose']['pose']['position']
            orientation = yaml_data['pose']['pose']['orientation']

            saved_pose = {
                'position': {
                    'x': position['x'],
                    'y': position['y'],
                    'z': position['z']
                },
                'orientation': {
                    'x': orientation['x'],
                    'y': orientation['y'],
                    'z': orientation['z'],
                    'w': orientation['w']
                }
            }
        rospy.loginfo(f"Loaded pose: {saved_pose}")
        return saved_pose
    except Exception as e:
        rospy.logerr(f"Failed to load saved pose: {e}")
        return None

def send_navigation_goal(goal_pose):
    """Sends a navigation goal to the move_base action server and waits for the result."""
    rospy.loginfo("Connecting to move_base action server...")
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server(rospy.Duration(5.0))
    rospy.loginfo("Connected to move_base server.")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = goal_pose['position']['x']
    goal.target_pose.pose.position.y = goal_pose['position']['y']
    goal.target_pose.pose.position.z = goal_pose['position']['z']

    goal.target_pose.pose.orientation.x = goal_pose['orientation']['x']
    goal.target_pose.pose.orientation.y = goal_pose['orientation']['y']
    goal.target_pose.pose.orientation.z = goal_pose['orientation']['z']
    goal.target_pose.pose.orientation.w = goal_pose['orientation']['w']

    rospy.loginfo(f"Sending goal: {goal}")
    client.send_goal(goal)

    rospy.loginfo("Waiting for the robot to reach the goal...")
    finished_within_time = client.wait_for_result(rospy.Duration(120.0))  # Wait up to 60 seconds
    if not finished_within_time:
        client.cancel_goal()
        rospy.logwarn("Timed out attempting to reach the goal.")
    else:
        state = client.get_state()
        if state == 3:
            rospy.loginfo("Goal reached successfully!")
        else:
            rospy.logwarn(f"Goal failed with state: {state}")

def main():
    rospy.init_node('navigate_to_saved_pose', anonymous=True)

    # Load saved map (optional for verification)
    rospy.loginfo(f"Using saved map from {map_file}")
    if not os.path.exists(map_file):
        rospy.logwarn(f"Map file {map_file} does not exist. Ensure navigation stack is launched with the correct map.")

    # Load saved pose
    saved_pose = load_saved_pose(odom_file)
    if saved_pose is None:
        rospy.logerr("Cannot proceed without a valid saved pose.")
        return

    # Send the navigation goal and wait for result
    send_navigation_goal(saved_pose)

if __name__ == "__main__":
    main()

