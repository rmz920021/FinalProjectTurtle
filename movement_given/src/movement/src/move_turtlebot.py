#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math

def rotate_to_angle(cmd_vel_pub, angle, angular_velocity=0.1):
    """
    Rotate the TurtleBot by a specified angle.
    Positive angles are counterclockwise; negative angles are clockwise.
    """
    rotate_duration = abs(angle) / angular_velocity
    twist = Twist()
    twist.angular.z = angular_velocity if angle > 0 else -angular_velocity

    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < rotate_duration and not rospy.is_shutdown():
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.1)

    # Stop rotation
    twist.angular.z = 0
    cmd_vel_pub.publish(twist)

def move_linear(cmd_vel_pub, distance, linear_velocity=0.05):
    """
    Move the TurtleBot forward by a specified distance.
    """
    move_duration = abs(distance) / linear_velocity
    twist = Twist()
    twist.linear.x = linear_velocity if distance > 0 else -linear_velocity

    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < move_duration and not rospy.is_shutdown():
        cmd_vel_pub.publish(twist)
        rospy.sleep(0.1)

    # Stop movement
    twist.linear.x = 0
    cmd_vel_pub.publish(twist)

def move_turtlebot():
    rospy.init_node('move_turtlebot_node', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Get user inputs
    x_translation = float(input("Enter x translation (meters): "))
    y_translation = float(input("Enter y translation (meters): "))
    z_rotation = float(input("Enter z rotation (degrees): "))

    # Convert inputs
    target_angle = math.atan2(y_translation, x_translation)
    distance = math.sqrt(x_translation**2 + y_translation**2)
    z_rotation_rad = math.radians(z_rotation)

    rospy.loginfo(f"Rotating to angle: {math.degrees(target_angle):.2f} degrees.")
    rotate_to_angle(cmd_vel_pub, target_angle)

    rospy.loginfo(f"Moving straight to target: {distance:.2f} meters.")
    move_linear(cmd_vel_pub, distance)

    final_rotation = z_rotation_rad - target_angle
    rospy.loginfo(f"Rotating to final orientation: {math.degrees(final_rotation):.2f} degrees.")
    rotate_to_angle(cmd_vel_pub, final_rotation)

    rospy.loginfo("Motion completed.")

if __name__ == '__main__':
    try:
        move_turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted.")
