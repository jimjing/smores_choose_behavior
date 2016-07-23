#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

class PointCloudLoader(object):
    def __init__(self, topic='/stat_filter/output'):
        self.pointcloud_gen = None
        scan_sub = rospy.Subscriber(topic, PointCloud2, self.on_scan)

    def on_scan(self, scan):
        rospy.loginfo("Got PointCloud2 ...")
        self.pointcloud_gen = pc2.read_points(scan, skip_nans=True, field_names=("x", "y", "z"))

    def getPointCloud(self):
        return self.pointcloud_gen

