#!/usr/bin/env python

import numpy as np
import time
from threading import Timer, Thread
from aenum import Enum
from subprocess import call
import pdb

import sys, os
from math import pi
sys.path.insert(0,"/home/{}/Embedded/ecosystem/smores_build/smores_reconfig/python/".format(os.environ['USER']))
import MissionPlayer
from name_map import *

import rospy
from tf import TransformListener
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String

from smores_choose_behavior.data_file_loader import DataFileLoader
from smores_choose_behavior.point_cloud_loader import PointCloudLoader
from smores_choose_behavior.visualizer import Visualizer
#from collision_detection.srv import *
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
        self.fetch_behavior = 0
        self._current_cmd = None
        self.MP = None
        self.b_name = "Tank"
        self.last_cmd_update_time = None
        self._timeout = 0.0
        self._robot_mission_state = None
        self._blob_reset_timer = None
        self._cmd_reset_timer = None
        self.tf_listener = None
        self.need_reconf = False
        self.done_reconf = False
        self.data_file_directory = ""

        self._current_data = None

        rospy.init_node('SMORES_Behavior_Planner', anonymous=True, log_level=rospy.DEBUG)
        rospy.loginfo("In BehaviorPlanner")
        self._initialize()
        self.main()

    def _initialize(self):

        if os.environ['USER'] == 'jim':
            self.data_file_directory = "/home/jim/Projects/smores_ros/src/smores_choose_behavior/data/"
        elif os.environ['USER'] == 'tarik':
            self.data_file_directory = "/home/tarik/catkin_ws/src/smores_choose_behavior/data/"
        self.DFL = DataFileLoader(data_file_directory = self.data_file_directory)
        self.DFL.loadAllData()
        self.Vis = Visualizer()

        #rospy.Subscriber("blobPt", Vector3, self.blobPt_callback, queue_size=1)
        rospy.Subscriber("/navigation_velocity_smoother/raw_cmd_vel", Twist, self.getCMD_callback, queue_size=1)
        rospy.Subscriber("/config_state", Int32, self.object_fetch_callback, queue_size=1)
        rospy.Subscriber("/reconf_status", String, self.reconf_status_callback, queue_size=1)

        self._current_data = self.DFL.data_dict[self.b_name]
        self.blob_coords = [0.0,0.0,0.0]
        self._timeout = 7.0
        self._current_cmd = Twist()
        self._blob_reset_timer = Timer(self._timeout, self.resetRobotState)
        self._cmd_reset_timer = Timer(self._timeout, self.resetRobotState)
        self.MP = MissionPlayer.MissionPlayer(self.data_file_directory + "{}/Behavior".format(self.b_name))
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

    def reconf_status_callback(self, data):
        self.done_reconf = True

    def object_fetch_callback(self, data):
        '''The callback for getting which behavior to run to fetch the object'''
        if data is not None:
            if data == -1:
                rospy.logerr("Cannot find a path to object!")
            elif data == 0:
                rospy.logwarn("Why do you send me a 0!?")
            else:
                rospy.logdebug("Recieve fetch behavior {}".format(data.data))
                self.fetch_behavior = data.data
                if self.fetch_behavior == 2:
                    self.need_reconf = True
                self.setRobotState(RobotMission.ToObject)

    def getCMD_callback(self, data):
        ''' Callback function for topic with location of object being tracked'''
        if data is not None:
            #rospy.loginfo("Linear is {} and Angular is {}.".format(data.linear.x, data.angular.z))
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
            #rospy.logdebug("Blob_coords is {}".format(self.blob_coords))
            #try:
            #    (world_tag_pose,world_tag_rot) = self.tf_listener.lookupTransform("camera_link","colorObj", rospy.Time(0))
            #    rospy.logdebug("Distance from the object is {}".format(world_tag_pose))
            #except:
            #    rospy.logdebug("Did not find object transform.")

            #if self._robot_mission_state == RobotMission.Explore:
            #    self.MP.playBehavior(self.b_name + "_diff.xml", para_mapping)
            #elif self._robot_mission_state == RobotMission.ToObject:
            if True:
                #if self.need_reconf:
                if True:
                    if not self.done_reconf:
                        # Do reconf here
                        rospy.loginfo("Reached reconf location. Stop!")
                        self.MP.playBehavior(self.b_name + "_Reconf.xml", {'para_L':0.0, 'para_R':0.0})
                        time.sleep(0.5)
                        self.MP.playBehavior(self.b_name + "_Reconf.xml", {'para_L':0.0, 'para_R':0.0})

                        self.MP.c.killCluster()
                        self.MP = None
                        time.sleep(1)

                        pub = rospy.Publisher('reconf_signal', String, queue_size=10)
                        while not self.done_reconf:
                            rospy.logdebug("Waiting for reconfiguration to finish.")
                            pub.publish(String("do"))
                            rate.sleep()

                        rospy.sleep(5)
                    if self.done_reconf:
                        # Run proboscis behaviors here
                        if self.MP is None:
                            self.MP = MissionPlayer.MissionPlayer(self.data_file_directory + "{}/Behavior".format("newPro"))

                        #if self.fetch_behavior == 2:
                        if True:
                            # Run tunnel
                            rospy.loginfo("Running tunnel...")
                            self.MP.playBehavior("newProPostReconf.xml", {})
                            rospy.sleep(2)
                            rospy.loginfo("Moving forward with proboscis...")
                            for i in xrange(10):
                                if i == 9:
                                    self.MP.c.mods[front_r].move.command_position('tilt', 10.0/180*pi,3)
                                self.MP.playBehavior("newProTunnel.xml", {'para_L':30,'para_R':30,'para_LB':-90,'para_RB':90 })

                                rospy.sleep(3)
                                self.MP.c.mods[front_r].mag.control('top', 'on')
                                rospy.sleep(0.1)

                            rospy.loginfo("Pickup object")
                            rospy.sleep(2)

                            rospy.loginfo("Moving backward with proboscis...")
                            for i in xrange(10):
                                self.MP.playBehavior("newProTunnel.xml", {'para_L':-30,'para_R':-30,'para_LB':90,'para_RB':-90 })
                                rospy.sleep(3)

                            rospy.loginfo("All done!")
                            return
                        elif self.fetch_behavior == 3:
                            # Run ledge
                            rospy.loginfo("Running ledge...")
                else:
                    # Run car behavior here
                    rospy.loginfo("Running car...")
                    self.MP.c.mods[front].move.command_position('tilt', -5.0/180*pi,1)
                    rospy.sleep(1)
                    for i in xrange(8):
                        if i == 7:
                            self.MP.c.mods[front].move.command_position('tilt', 10.0/180*pi,3)
                        self.MP.playBehavior(self.b_name + "_diff.xml", {"para_L":-90.0, "para_R":90.0})
                        rospy.sleep(0.5)
                        self.MP.c.mods[front].mag.control('top', 'on')
                        rospy.sleep(0.5)
                    rospy.loginfo("Pickup object")
                    rospy.sleep(5)

                    for i in xrange(8):
                        self.MP.playBehavior(self.b_name + "_diff.xml", {"para_L":90.0, "para_R":-90.0})
                    return
            else:
                rospy.logwarn("Have not got a command for more than {} seconds. System idling.".format(self._timeout))

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

