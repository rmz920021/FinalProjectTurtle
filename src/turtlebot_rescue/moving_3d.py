#!/usr/bin/env python3

import rospy
import os
import math
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np

odom_file = os.path.expanduser("~/Desktop/FinalProjectTurtle/src/turtlebot_rescue/saved_data/robot_odom.txt")

class Simple3DNavigator:
    def __init__(self, goal_pose):
        # goal_pose is a dict with position (x,y,z) and orientation (x,y,z,w)
        self.goal_pose = goal_pose
        self.current_pose = None
        self.laser_data = None
        self.pose_sub = rospy.Subscriber("/rtabmap/localization_pose", PoseWithCovarianceStamped, self.pose_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(10)  # 10 Hz

        # Navigation parameters
        self.position_threshold = 0.1  # meters
        self.yaw_threshold = 0.05      # radians
        self.linear_speed = 0.1        # m/s
        self.angular_speed = 0.2       # rad/s

        # Obstacle avoidance parameters
        self.safe_distance = 0.4  # Stop if obstacle is closer than 40 cm directly ahead

    def pose_callback(self, msg):
        self.current_pose = msg

    def scan_callback(self, msg):
        self.laser_data = msg

    def quaternion_to_yaw(self, qx, qy, qz, qw):
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def distance_to_goal(self):
        if self.current_pose is None:
            return None
        dx = self.goal_pose['x'] - self.current_pose.pose.pose.position.x
        dy = self.goal_pose['y'] - self.current_pose.pose.pose.position.y
        dz = self.goal_pose['z'] - self.current_pose.pose.pose.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def angle_to_goal(self):
        if self.current_pose is None:
            return None
        goal_yaw = self.quaternion_to_yaw(self.goal_pose['ox'], self.goal_pose['oy'],
                                          self.goal_pose['oz'], self.goal_pose['ow'])
        current_yaw = self.quaternion_to_yaw(self.current_pose.pose.pose.orientation.x,
                                             self.current_pose.pose.pose.orientation.y,
                                             self.current_pose.pose.pose.orientation.z,
                                             self.current_pose.pose.pose.orientation.w)
        angle_diff = goal_yaw - current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        return angle_diff
    
    def locate(self):
        """Locate where the robot is by exploring the environment until it receives localization data."""
        rospy.loginfo("Exploring to locate the robot's position...")

        exploration_duration = 60  # Time to explore (in seconds)
        start_time = rospy.Time.now()

        twist = Twist()
        pose_received = False

        while not rospy.is_shutdown():
            # Check if pose is received
            if self.current_pose is not None:
                rospy.loginfo("Localization successful! Pose received.")
                pose_received = True
                break

            # Exploration strategy: random rotations and slight movements
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            if elapsed_time > exploration_duration:
                rospy.logerr("Timeout exceeded: Unable to locate the robot.")
                break

            # Rotate randomly
            twist.angular.z = np.random.choice([-self.angular_speed, self.angular_speed])
            twist.linear.x = self.linear_speed * 0.5  # Move slightly forward

            self.cmd_pub.publish(twist)
            self.rate.sleep()

        # Stop the robot after locating or timeout
        self.cmd_pub.publish(Twist())

        if pose_received:
            rospy.loginfo("Robot successfully localized.")
        else:
            rospy.logerr("Localization failed. Check the /rtabmap/localization_pose topic.")
        
    def front_obstacle_distance(self):
        if self.laser_data is None:
            return None
        ranges = self.laser_data.ranges
        if len(ranges) == 0:
            return None
        center_index = len(ranges) // 2
        window_size = 10
        front_ranges = ranges[center_index-window_size:center_index+window_size]
        # Filter out invalid ranges
        front_ranges = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]
        if len(front_ranges) == 0:
            # No valid readings, assume no obstacle
            return None
        return min(front_ranges)

    def run(self):
        rospy.loginfo("Waiting for localization and scan data...")
        while not rospy.is_shutdown():
            if self.current_pose is not None and self.laser_data is not None:
                break
            self.rate.sleep()
        rospy.loginfo("locate itself to see where it is")
        self.locate()
        
        rospy.loginfo("Navigating to saved 3D pose with obstacle avoidance...")

        while not rospy.is_shutdown():
            dist = self.distance_to_goal()
            angle = self.angle_to_goal()

            if dist is None or angle is None:
                self.rate.sleep()
                continue

            cmd = Twist()

            if dist < self.position_threshold:
                # Align orientation
                if abs(angle) < self.yaw_threshold:
                    rospy.loginfo("Goal reached!")
                    break
                else:
                    # Rotate in place to match orientation
                    cmd.angular.z = self.angular_speed if angle > 0 else -self.angular_speed
                    self.cmd_pub.publish(cmd)

            else:
                dx = self.goal_pose['x'] - self.current_pose.pose.pose.position.x
                dy = self.goal_pose['y'] - self.current_pose.pose.pose.position.y
                heading = math.atan2(dy, dx)

                current_yaw = self.quaternion_to_yaw(self.current_pose.pose.pose.orientation.x,
                                                     self.current_pose.pose.pose.orientation.y,
                                                     self.current_pose.pose.pose.orientation.z,
                                                     self.current_pose.pose.pose.orientation.w)
                yaw_error = heading - current_yaw
                yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

                # Obstacle handling (disabled here for simplicity)
                # if obstacle_dist is not None and obstacle_dist < self.safe_distance:
                #     ...
                # else:
                if abs(yaw_error) > 0.1:
                    cmd.angular.z = self.angular_speed if yaw_error > 0 else -self.angular_speed
                else:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = 0.0

                self.cmd_pub.publish(cmd)

            self.rate.sleep()

        # Stop the robot
        self.cmd_pub.publish(Twist())
        rospy.loginfo("Done.")


def parse_odom_file(file_path):
    position = {}
    orientation = {}

    with open(file_path, 'r') as f:
        lines = f.readlines()

    reading_position = False
    reading_orientation = False

    for line in lines:
        line_stripped = line.strip()
        if line_stripped.startswith("position:"):
            reading_position = True
            reading_orientation = False
            continue

        if line_stripped.startswith("orientation:"):
            reading_orientation = True
            reading_position = False
            continue

        if reading_position:
            if line_stripped.startswith('x:'):
                position['x'] = float(line_stripped.split(':')[1])
            elif line_stripped.startswith('y:'):
                position['y'] = float(line_stripped.split(':')[1])
            elif line_stripped.startswith('z:'):
                position['z'] = float(line_stripped.split(':')[1])

        if reading_orientation:
            if line_stripped.startswith('x:'):
                orientation['x'] = float(line_stripped.split(':')[1])
            elif line_stripped.startswith('y:'):
                orientation['y'] = float(line_stripped.split(':')[1])
            elif line_stripped.startswith('z:'):
                orientation['z'] = float(line_stripped.split(':')[1])
            elif line_stripped.startswith('w:'):
                orientation['w'] = float(line_stripped.split(':')[1])

    if ('x' not in position or 'y' not in position or 
        'z' not in position or 'x' not in orientation or
        'y' not in orientation or 'z' not in orientation or
        'w' not in orientation):
        raise ValueError("Failed to parse position and orientation from odometry file.")

    return position, orientation

def main():
    rospy.init_node('go_to_saved_pose_3d', anonymous=True)

    rospy.loginfo("Reading saved odometry from file...")
    try:
        position, orientation = parse_odom_file(odom_file)
    except Exception as e:
        rospy.logerr(f"Failed to parse odometry file: {e}")
        return

    goal_pose = {
        'x': position['x'],
        'y': position['y'],
        'z': position['z'],
        'ox': orientation['x'],
        'oy': orientation['y'],
        'oz': orientation['z'],
        'ow': orientation['w']
    }

    # --- Added Code to Publish Pose for Visualization ---
    pose_pub = rospy.Publisher("/test_pose", PoseStamped, queue_size=1)
    rospy.sleep(1.0)  # Wait a moment for the publisher to register

    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "map"
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.position.x = goal_pose['x']
    pose_msg.pose.position.y = goal_pose['y']
    pose_msg.pose.position.z = goal_pose['z']
    pose_msg.pose.orientation.x = goal_pose['ox']
    pose_msg.pose.orientation.y = goal_pose['oy']
    pose_msg.pose.orientation.z = goal_pose['oz']
    pose_msg.pose.orientation.w = goal_pose['ow']

    # Publish the pose for visualization in RViz
    pose_pub.publish(pose_msg)
    rospy.loginfo("Published the goal pose for visualization in RViz.")

    # -----------------------------------------------------

    navigator = Simple3DNavigator(goal_pose)
    navigator.run()

if __name__ == "__main__":
    main()

