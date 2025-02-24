#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg

rospy.init_node("laserscan_to_pointcloud")

lp = lg.LaserProjection()

pc_pub = rospy.Publisher("converted_pc", PointCloud2, queue_size=1)

def scan_cb(msg):

    # Convert the message of type LaserScan to a PointCloud2
    pc2_msg = lp.projectLaser(msg)
    
    # Publish it on a new topic
    pc_pub.publish(pc2_msg)



rospy.Subscriber("/dji0/laser_scan", LaserScan, scan_cb, queue_size=1)
rospy.spin()
