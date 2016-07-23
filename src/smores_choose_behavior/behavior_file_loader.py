#!/usr/bin/env python

import os
import logging
import xml.etree.ElementTree as ET
import rospy
from geometry_msgs.msg import Pose, PoseArray

class BehaviorFileLoader(object):
    def __init__(self, behavior_file_directory=""):

        self.behavior_file_directory = behavior_file_directory
        self.behavior_dict = {}

    def loadBehaviorFile(self, file_path):
        logging.info("Loading behavior file: {}".format(file_path))
        with open(file_path,'r') as f:
            self.behavior_dict[file_path] = self._parseBehaviorDataToPath(f)
        return self.behavior_dict[file_path]

    def loadAllBehaviorFiles(self, behavior_file_directory=""):
        if behavior_file_directory == "":
            behavior_file_directory = self.behavior_file_directory

        behavior_file_list = []
        for (dirpath, dirnames, filenames) in os.walk(behavior_file_directory):
            behavior_file_list.extend(filenames)
            break

        for f in behavior_file_list:
            self.loadBehaviorFile(os.path.join(self.behavior_file_directory,f))

    def _parseBehaviorDataToPath(self, data):
        tree = ET.parse(data)
        root = tree.getroot() # root is everything in the <behavior> tag
        xRobotStates = root[0]

        path = []

        for xRobotState in xRobotStates:
            xModuleStates = xRobotState.find('ModuleStates')
            for xModuleState in xModuleStates:
                p = Pose()
                position = xModuleState.find('position')
                p.position.x = float(position.find('x').text)/10
                p.position.y = float(position.find('z').text)/10
                p.position.z = float(position.find('y').text)/10
                rotation = xModuleState.find('rotation')
                p.orientation.x = float(rotation.find('x').text)
                p.orientation.y = float(rotation.find('y').text)
                p.orientation.z = float(rotation.find('z').text)
                p.orientation.w = float(rotation.find('w').text)
                path.append(p)

        return path
