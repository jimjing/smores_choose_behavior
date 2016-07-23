#!/usr/bin/env python

import numpy as np

import rospy
from smores_choose_behavior.behavior_file_loader import BehaviorFileLoader
from smores_choose_behavior.point_cloud_loader import PointCloudLoader
from smores_choose_behavior.visualizer import Visualizer
from collision_detection.srv import *
from geometry_msgs.msg import Pose, PoseArray

class BehaviorPlanner(object):
    def __init__(self):
        self.BFL = None
        self.PCL = None
        self.Vis = None
        self._collision_tolerance = 0.06 # meters

        rospy.init_node('SMORES_Behavior_Planner', anonymous=True, log_level=rospy.DEBUG)
        rospy.loginfo("In BehaviorPlanner")
        self._initialize()
        self.main()

    def _initialize(self):
        self.BFL = BehaviorFileLoader(behavior_file_directory = "/home/jim/Projects/smores_ros/src/smores_choose_behavior/data/behavior_files")
        self.BFL.loadAllBehaviorFiles()
        self.Vis = Visualizer()
        #self.PCL = PointCloudLoader(topic='/cloud_pcd')


    def main(self):
        for behavior_file, path in self.BFL.behavior_dict.iteritems():
            rospy.loginfo("Checking behavior {}".format(behavior_file))
            self._checkCollisionBehaviorWithPointCloud(path)

        self.Vis.stop()

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

