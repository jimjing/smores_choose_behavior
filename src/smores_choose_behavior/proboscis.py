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

class Proboscis(object):
    def __init__(self):
        self.blob_coords = None # [x,y,z]
        self.MP = None
        self.b_name = "Proboscis"

        rospy.init_node('Proboscis', anonymous=True, log_level=rospy.DEBUG)
        rospy.loginfo("In Proboscis")
        self._initialize()
        self.main()

    def _initialize(self):
        rospy.Subscriber("blobPt", Vector3, self.blobPt_callback, queue_size=1)

        self.blob_coords = [0.0,0.0,0.0]
        self.MP = MissionPlayer.MissionPlayer("/home/jim/Projects/smores_ros/src/smores_choose_behavior/data/{}/Behavior".format(self.b_name))

    def main(self):
        self.run()

    def blobPt_callback(self, data):
        ''' Callback function for topic with location of object being tracked'''
        if data is not None:
            self.blob_coords = [data.x, data.y, data.z]

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()
            rospy.logdebug("Blob_coords is {}".format(self.blob_coords))
            if self.blob_coords[2] > 0:
                self.MP.playBehavior("drop", {"para_V":30})

            if self.blob_coords[2]>2000.0:
                rospy.loginfo("Picking up")
                self.MP.playBehavior("drop", {"para_V":40})
                time.sleep(0.1)
                self.MP.playBehavior("drop", {"para_V":40})
                time.sleep(0.1)
                self.MP.playBehavior("drop", {"para_V":40})
                time.sleep(0.1)
                self.MP.playBehavior("drop", {"para_V":40})
                time.sleep(0.1)

                self.MP.playBehavior("lift", {"para_V":-60})
                time.sleep(0.1)
                self.MP.playBehavior("lift", {"para_V":-60})
                time.sleep(0.1)
                self.MP.playBehavior("lift", {"para_V":-60})
                time.sleep(0.1)
                self.MP.playBehavior("lift", {"para_V":-60})
                return

