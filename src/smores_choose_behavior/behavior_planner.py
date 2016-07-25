#!/usr/bin/env python

import numpy as np

import rospy
from smores_choose_behavior.data_file_loader import DataFileLoader
from smores_choose_behavior.point_cloud_loader import PointCloudLoader
from smores_choose_behavior.visualizer import Visualizer
from collision_detection.srv import *
from geometry_msgs.msg import Pose, PoseArray

class BehaviorPlanner(object):
    def __init__(self):
        self.DFL = None
        self.PCL = None
        self.Vis = None
        self._collision_tolerance = 0.06 # meters

        self._current_configuration = None
        self._current_behavior_list = []
        self._current_path_list = []

        rospy.init_node('SMORES_Behavior_Planner', anonymous=True, log_level=rospy.DEBUG)
        rospy.loginfo("In BehaviorPlanner")
        self._initialize()
        self.main()

    def _initialize(self):
        self.DFL = DataFileLoader(data_file_directory = "/home/jim/Projects/smores_ros/src/smores_choose_behavior/data")
        self.DFL.loadAllData()
        self.Vis = Visualizer()
        #self.PCL = PointCloudLoader(topic='/cloud_pcd')

    def shutdown(self):
        self.Vis.stop()

    def main(self):
        self._checkAllPaths()
        self.shutdown()

    def _checkAllPaths(self):
        for data_dir, data in self.DFL.data_dict.iteritems():
            rospy.loginfo("Checking data directory {!r}".format(data_dir))
            for path_file_name, path in data.path_dict.iteritems():
                rospy.loginfo("Checking path {!r}".format(path_file_name))
                self._checkCollisionBehaviorWithPointCloud(path)

    def _checkCollisionBehaviorWithPointCloud(self, behavior_path):
        rospy.loginfo("Sending collision request ...")
        rospy.wait_for_service('path_env_collision')
        try:
            path_env_collision_srv = rospy.ServiceProxy('path_env_collision', CheckCollision)

            p_array = PoseArray()
            p_array.header.frame_id = "camera_link";
            p_array.header.stamp = rospy.Time();
            for pt in behavior_path:
                p_array.poses.append(pt)

            self.Vis.setPubPath(p_array.poses)
            resp = path_env_collision_srv(p_array)
            print "Get collision ... {}".format(resp.collide)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        return False

