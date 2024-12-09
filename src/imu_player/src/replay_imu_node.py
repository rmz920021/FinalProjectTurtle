#!/usr/bin/env python

import rospy
import csv
import os
from geometry_msgs.msg import Twist
import time

# File path for the IMU data
data_directory = os.path.join(os.path.dirname(__file__), '../../data')
file_path = os.path.join(data_directory, 'imu_data.csv')

def replay_trajectory():
    """
    Reads IMU data from a CSV file and replays the trajectory
    by publishing velocities to /cmd_vel.
    """
    # Initialize the ROS node
    rospy.init_node('replay_imu_node', anonymous=True)

    # Publisher for /cmd_vel
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Ensure the file exists
    if not os.path.exists(file_path):
        rospy.logerr(f"File {file_path} does not exist. Exiting.")
        return

    rospy.loginfo(f"Replaying trajectory from {file_path}")

    try:
        # Open the CSV file for reading
        with open(file_path, mode='r') as csv_file:
            reader = csv.reader(csv_file)
            header = next(reader)  # Skip the header row
            
            previous_time = None

            for row in reader:
                # Read the data from each row
                timestamp, linear_velocity, angular_velocity = float(row[0]), float(row[1]), float(row[2])

                # Calculate the time difference for this motion
                if previous_time is None:
                    # For the first row, initialize previous_time
                    previous_time = timestamp
                    continue

                time_diff = timestamp - previous_time
                previous_time = timestamp

                # Create the Twist message
                twist = Twist()
                twist.linear.x = linear_velocity
                twist.angular.z = angular_velocity

                # Publish the velocity command for the time difference
                start_time = rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec() - start_time < time_diff:
                    cmd_vel_pub.publish(twist)
                    rospy.sleep(0.1)  # Publish at 10 Hz

                rospy.loginfo(f"Moved: linear_vel={linear_velocity:.2f}, angular_vel={angular_velocity:.2f}, duration={time_diff:.2f}s")

            # Stop the robot after completing the trajectory
            cmd_vel_pub.publish(Twist())
            rospy.loginfo("Trajectory replay completed.")

    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

if __name__ == '__main__':
    try:
        replay_trajectory()
    except rospy.ROSInterruptException:
        rospy.loginfo("Replay interrupted.")