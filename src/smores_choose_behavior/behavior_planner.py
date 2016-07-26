#!/usr/bin/env python

import numpy as np
import pdb

import rospy
from tf import TransformListener

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
        self.tf = None
        self._collision_tolerance = 0.06 # meters

        self._current_data = None
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
        self.tf = TransformListener()
        self._current_data = self.DFL.data_dict["ThreeMCar"]
        #self.PCL = PointCloudLoader(topic='/cloud_pcd')

    def shutdown(self):
        self.Vis.stop()

    def main(self):
        self.run()
        self.shutdown()

    def run(self):

        while not rospy.is_shutdown():
            self.rankPathWithData(self._current_data)

            if self._current_path_list == []:
                rospy.logerr("The path list is empty!")
                return

            for path_file_name in self._current_path_list:
                rospy.loginfo("Checking path {!r}".format(path_file_name))
                self._checkCollisionBehaviorWithPointCloud(self._current_data.path_dict[path_file_name])

    def getGoalPose(self):
        self.tf.waitForTransform("/camera_link", "/test", rospy.Time(), rospy.Duration(4.0))
        if self.tf.frameExists("/camera_link") and self.tf.frameExists("/test"):
            t = self.tf.getLatestCommonTime("/camera_link", "/test")
            position, quaternion = self.tf.lookupTransform("/camera_link", "/test", t)
            return position
        else:
            pdb.set_trace()
            rospy.logerr("Cannot find transform !!!")
            return None

    def rankPathWithData(self, data=None):
        if data is None:
            data = self._current_data

        pose = self.getGoalPose()
        if pose is None:
            self._current_path_list = []
            self._current_behavior_list = []
            return

        dist_list = []
        path_list = []
        for path_file_name, path in data.path_dict.iteritems():
            pt = [path[-1].position.x, path[-1].position.y, path[-1].position.z]
            dist_list.append(self.calcDist(pose, pt))
            path_list.append(path_file_name)

        sorted_indices = [i[0] for i in sorted(enumerate(dist_list), key=lambda x:x[1])]

        self._current_path_list = [path_list[i] for i in sorted_indices]
        self._current_behavior_list = [x.lstrip("R_") for x in self._current_path_list]

    def calcDist(self, pt1, pt2):
        return np.linalg.norm(np.array(pt1)-np.array(pt2))

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

