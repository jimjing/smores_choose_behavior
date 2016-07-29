#!/usr/bin/env python

import numpy as np
import pdb

import sys
sys.path.insert(0,"/home/jim/Embedded/ecosystem/smores_build/smores_reconfig/python/")
import MissionPlayer

import rospy
from tf import TransformListener
from geometry_msgs.msg import Twist

from smores_choose_behavior.data_file_loader import DataFileLoader
from smores_choose_behavior.point_cloud_loader import PointCloudLoader
from smores_choose_behavior.visualizer import Visualizer
from collision_detection.srv import *
from geometry_msgs.msg import Pose, PoseArray, Vector3

class BehaviorPlanner(object):
    def __init__(self):
        self.DFL = None
        self.PCL = None
        self.Vis = None
        self.blob_coords = None # [x,y,z]
        self._current_cmd = None
        self.MP = None
        self.b_name = "SevenMCar"

        self._current_data = None

        rospy.init_node('SMORES_Behavior_Planner', anonymous=True, log_level=rospy.DEBUG)
        rospy.loginfo("In BehaviorPlanner")
        self._initialize()
        self.main()

    def _initialize(self):
        self.DFL = DataFileLoader(data_file_directory = "/home/jim/Projects/smores_ros/src/smores_choose_behavior/data")
        self.DFL.loadAllData()
        self.Vis = Visualizer()

        rospy.Subscriber("blobPt", Vector3, self.blobPt_callback)
        rospy.Subscriber("/mobile_base/commands/velocity", Twist, self.getCMD_callback)

        self._current_data = self.DFL.data_dict[self.b_name]
        self.blob_coords = [0.0,0.0,0.0]
        self._current_cmd = Twist()
        self.MP = MissionPlayer.MissionPlayer("/home/jim/Projects/smores_ros/src/smores_choose_behavior/data/{}/Behavior".format(self.b_name))
        #self.PCL = PointCloudLoader(topic='/cloud_pcd')

    def shutdown(self):
        self.Vis.stop()

    def main(self):
        self.run()
        self.shutdown()

    def blobPt_callback(self, data):
        ''' Callback function for topic with location of object being tracked'''
        self.blob_coords = [data.x, data.y, data.z]

    def getCMD_callback(self, data):
        ''' Callback function for topic with location of object being tracked'''
        self._current_cmd = data

    def input2Output(self, para_dict):

        output_mapping = {}

        for input_name, input_value in para_dict["input"].iteritems():
            exec_str = "{}={}".format(input_name, input_value)
            exec(exec_str)
        for output_name, output_value in para_dict["output"].iteritems():
            exec("{}={}".format(output_name, output_value))
            output_mapping[output_name] = eval(output_name)
        return output_mapping

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            para_mapping = self.input2Output(self._current_data.para_dict[self.b_name + "Diff.xml"])
            rospy.logdebug(para_mapping)
            rate.sleep()
            self.MP.playBehavior(self.b_name + "Diff.xml", para_mapping)

    def getGoalPose(self):
        return self.blob_coords

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
            rospy.loginfo("Get collision ... {}".format(resp.collide))

        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: {}".format(e))

        return resp.collide

