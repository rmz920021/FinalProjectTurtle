#!/usr/bin/env python

import rospy
import csv
import os
from geometry_msgs.msg import Twist

# File path for storing data
data_directory = os.path.join(os.path.dirname(__file__), '../../data')
os.makedirs(data_directory, exist_ok=True)
file_path = os.path.join(data_directory, 'imu_data.csv')

# Define the CSV writer as a global variable
csv_file = open(file_path, mode='w', newline='')
writer = csv.writer(csv_file)

# Write the CSV header
writer.writerow(['timestamp', 'linear_velocity', 'angular_velocity'])

def cmd_vel_callback(msg):
    """
    Callback function to record linear and angular velocities.
    """
    # Get the current time
    timestamp = rospy.Time.now().to_sec()
    # Record linear and angular velocities
    linear_velocity = msg.linear.x
    angular_velocity = msg.angular.z

    # Write data to CSV
    writer.writerow([timestamp, linear_velocity, angular_velocity])
    csv_file.flush()  # Ensure data is written to the file immediately
    rospy.loginfo(f"Recorded: time={timestamp:.2f}, linear_vel={linear_velocity:.2f}, angular_vel={angular_velocity:.2f}")

def main():
    """
    Main function to initialize the node and start recording.
    """
    rospy.init_node('record_cmd_vel_node', anonymous=True)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.loginfo("Recording /cmd_vel data...")
    rospy.spin()
    csv_file.close()  # Close the file when the node shuts down

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        csv_file.close()
