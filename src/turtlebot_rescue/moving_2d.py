#!/usr/bin/env python3

import rospy
import json
import os
import yaml
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler

# File paths for saved data
map_file = os.path.expanduser("~/Desktop/FinalProjectTurtle/src/turtlebot_rescue/saved_data/slam_map.yaml")
odom_file = os.path.expanduser("~/Desktop/FinalProjectTurtle/src/turtlebot_rescue/saved_data/robot_odom.txt")

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

            rospy.loginfo(f"{position['x']}")

           
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

def publish_goal(goal_pose):
    """Publishes a goal for the robot to navigate to."""
    rospy.loginfo("Publishing goal to move_base...")

    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # Wait for the publisher to be ready
    rospy.sleep(2)

    # Create PoseStamped message
    goal_msg = PoseStamped()
    goal_msg.header.frame_id = "map"
    goal_msg.header.stamp = rospy.Time.now()

    goal_msg.pose.position.x = goal_pose['position']['x']
    goal_msg.pose.position.y = goal_pose['position']['y']
    goal_msg.pose.orientation.x = goal_pose['orientation']['x']
    goal_msg.pose.orientation.y = goal_pose['orientation']['y']
    goal_msg.pose.orientation.z = goal_pose['orientation']['z']
    goal_msg.pose.orientation.w = goal_pose['orientation']['w']

    goal_pub.publish(goal_msg)
    rospy.loginfo(f"Goal published: {goal_msg.pose}")

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

    # Publish the navigation goal
    publish_goal(saved_pose)

    rospy.loginfo("Waiting for the robot to reach the goal...")
    rospy.spin()

if __name__ == "__main__":
    main()