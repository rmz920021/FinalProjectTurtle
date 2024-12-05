#!/usr/bin/env python

import rospy
import os
import json
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty

class SaveMappingLocation:
    def __init__(self):
        rospy.init_node('save_mapping_location', anonymous=True)

        self.odom_data = None
        self.pointcloud_data = None

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)

        # Wait for map_saver service (if using SLAM or similar)
        rospy.wait_for_service('map_saver')
        self.save_map_service = rospy.ServiceProxy('map_saver', Empty)

    def odom_callback(self, msg):
        self.odom_data = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            }
        }

    def pointcloud_callback(self, msg):
        self.pointcloud_data = msg

    def save_data(self):
        rospy.loginfo("Saving map, odometry, and pointcloud data...")

        # Save map (if applicable)
        try:
            self.save_map_service()
            rospy.loginfo("Map saved successfully!")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to save map: {e}")

        # Save odometry and pointcloud data
        if self.odom_data and self.pointcloud_data:
            data_path = os.path.expanduser("~/FinalProjectTurtle/src/turtlebot_rescue/saved_data")
            os.makedirs(data_path, exist_ok=True)

            with open(os.path.join(data_path, 'odometry.json'), 'w') as odom_file:
                json.dump(self.odom_data, odom_file)

            # Save pointcloud as a ROS bag or other format
            pointcloud_path = os.path.join(data_path, 'pointcloud.bag')
            rospy.loginfo(f"Saving pointcloud to {pointcloud_path}")
            with open(pointcloud_path, 'wb') as pc_file:
                pc_file.write(self.pointcloud_data.data)

            rospy.loginfo("Odometry and pointcloud data saved successfully!")
        else:
            rospy.logwarn("Odometry or pointcloud data is missing.")

if __name__ == '__main__':
    save_node = SaveMappingLocation()
    rospy.sleep(10)  # Allow time to collect data
    save_node.save_data()