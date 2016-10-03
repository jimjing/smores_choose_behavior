#!/usr/bin/env python

import numpy as np
import time
from threading import Timer, Thread
from aenum import Enum
from subprocess import call
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

class RobotMission(Enum):
    Explore = 1
    ToObject = 2
    Idle = 3

class BehaviorPlanner(object):
    def __init__(self):
        self.DFL = None
        self.PCL = None
        self.Vis = None
        self.blob_coords = None # [x,y,z]
        self._current_cmd = None
        self.MP = None
        self.b_name = "Tank"
        self.last_cmd_update_time = None
        self._timeout = 0.0
        self._robot_mission_state = None
        self._blob_reset_timer = None
        self._cmd_reset_timer = None
        self.tf_listener = None

        self._current_data = None

        rospy.init_node('SMORES_Behavior_Planner', anonymous=True, log_level=rospy.DEBUG)
        rospy.loginfo("In BehaviorPlanner")
        self._initialize()
        self.main()

    def _initialize(self):
        self.DFL = DataFileLoader(data_file_directory = "/home/jim/Projects/smores_ros/src/smores_choose_behavior/data")
        self.DFL.loadAllData()
        self.Vis = Visualizer()

        rospy.Subscriber("blobPt", Vector3, self.blobPt_callback, queue_size=1)
        rospy.Subscriber("/navigation_velocity_smoother/raw_cmd_vel", Twist, self.getCMD_callback, queue_size=1)

        self._current_data = self.DFL.data_dict[self.b_name]
        self.blob_coords = [0.0,0.0,0.0]
        self._timeout = 7.0
        self._current_cmd = Twist()
        self._blob_reset_timer = Timer(self._timeout, self.resetRobotState)
        self._cmd_reset_timer = Timer(self._timeout, self.resetRobotState)
        self.MP = MissionPlayer.MissionPlayer("/home/jim/Projects/smores_ros/src/smores_choose_behavior/data/{}/Behavior".format(self.b_name))
        self._robot_mission_state = RobotMission.Idle
        self.tf_listener = TransformListener()
        #self.PCL = PointCloudLoader(topic='/cloud_pcd')
       #t = Thread(target=self.fakeSignal)
       #t.setDaemon(True)
       #t.start()

    def fakeSignal(self):
        while not rospy.is_shutdown():
            cmd = raw_input("e for explore, o for object: ")
            if cmd  == 'e':
                self.getCMD_callback(None)
            elif cmd  == 'o':
                self.blobPt_callback(None)
            cmd = ""

    def setRobotState(self, state):
        if state == self._robot_mission_state:
            return
        rospy.loginfo("Switching robot mission from {} to {}.".format(self._robot_mission_state, state))
        self._robot_mission_state = state

    def resetRobotState(self):
        rospy.loginfo("Reset state")
        self.setRobotState(RobotMission.Idle)

    def shutdown(self):
        self.Vis.stop()

    def main(self):
        self.run()
        self.shutdown()

    def blobPt_callback(self, data):
        ''' Callback function for topic with location of object being tracked'''
        if data is not None:
            self.blob_coords = [data.x, data.y, data.z]
        self.setRobotState(RobotMission.ToObject)
        if self._blob_reset_timer.is_alive():
            self._blob_reset_timer.cancel()
            time.sleep(0.1)
        self._blob_reset_timer = Timer(self._timeout, self.resetRobotState)
        self._blob_reset_timer.start()

    def getCMD_callback(self, data):
        ''' Callback function for topic with location of object being tracked'''
        if data is not None:
            if (data.linear.x != 0.0 or data.angular.z != 0.0):
                self._current_cmd = data

        if self._robot_mission_state != RobotMission.ToObject:
            self.setRobotState(RobotMission.Explore)

            if self._cmd_reset_timer.is_alive():
                self._cmd_reset_timer.cancel()
                time.sleep(0.1)
            self._cmd_reset_timer= Timer(self._timeout, self.resetRobotState)
            self._cmd_reset_timer.start()

    def clip(self, min_v, max_v, data):
        if data == 0.0:
            return data
        if abs(data) > max_v:
            return max_v * np.sign(data)
        if abs(data) < min_v:
            return min_v * np.sign(data)
        return data

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
            para_mapping = self.input2Output(self._current_data.para_dict[self.b_name + "_diff.xml"])
            rospy.logdebug(para_mapping)
            rate.sleep()
            rospy.logdebug("Blob_coords is {}".format(self.blob_coords))
            try:
                (world_tag_pose,world_tag_rot) = self.tf_listener.lookupTransform("camera_link","colorObj", rospy.Time(0))
                rospy.logdebug("Distance from the object is {}".format(world_tag_pose))
            except:
                rospy.logdebug("Did not find object transform.")
            if self._robot_mission_state != RobotMission.Idle:
                self.MP.playBehavior(self.b_name + "_diff.xml", para_mapping)
            else:
                rospy.logwarn("Have not got a command for more than {} seconds. System idling.".format(self._timeout))

            if self.blob_coords[1] > 0.0 and self.blob_coords[1] < 130.0 and self.blob_coords[0] < 100.0 and self.blob_coords[0] > -100.0:
                rospy.loginfo("Reached reconf location. Stop!")
                self.MP.playBehavior(self.b_name + "PreReconf.xml", para_mapping)
                time.sleep(0.5)
                self.MP.playBehavior(self.b_name + "PreReconf.xml", para_mapping)

                self.MP.c.killCluster()
                time.sleep(1)
                #pdb.set_trace()
                #call(["rosrun", "path_follow", "path_follow_node.py"])
                return


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

