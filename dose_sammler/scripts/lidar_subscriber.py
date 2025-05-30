#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

# Define the variable for the latest scan:
latest_scan_data = None


# Define the callback function for the LIDAR scan:
def callback(scan):
    global latest_scan_data
    latest_scan_data = scan


# Init the LIDAR subscriber:
def init_lidar_subscriber():
    rospy.loginfo("init lidar")
    rospy.Subscriber('/scan', LaserScan, callback)


# Get the last LIDAR scan data:
def get_latest_scan():
    rospy.loginfo("get scan data")
    return latest_scan_data


# Start the node:
if __name__ == '__main__':
    rospy.init_node('lidar_node', anonymous=True)
    init_lidar_subscriber()
    rospy.spin()