#!/usr/bin/env python

import logging
from smores_choose_behavior.behavior_file_loader import BehaviorFileLoader

logging.info("Testing BehaviorFileLoader ...")
BFL = BehaviorFileLoader(behavior_file_directory = "/home/jim/Projects/smores_ros/src/smores_choose_behavior/data/behavior_files")
BFL.loadAllBehaviorFiles()
