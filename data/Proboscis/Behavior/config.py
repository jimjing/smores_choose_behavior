# config.py
import sys
sys.path.append('../..')
import name_map

"""
Simulator names:

4
1-0-2-5-6
3
"""

from numpy import pi

angle = 90
#ModuleMap = {'SMORES_0':13, 'SMORES_1':12, 'SMORES_2':14, 'SMORES_3':3,
#             'SMORES_4':16, 'SMORES_5':9,'SMORES_6':8,}
ModuleMap = {'SMORES_0':middle, 'SMORES_1':back_middle, 'SMORES_2':front_middle, 'SMORES_3':back_right,
             'SMORES_4':back_left, 'SMORES_5':front_right,'SMORES_6':front_left,}
NeutralPositions = {}

disabledDof=['pan']
speed_scale=0.7
behaviorFiles = {
   'ProboscisOverBar.xml':'ProboscisOverBar.xml',
   'ProboscisPickup.xml':'ProboscisPickup.xml',
   'ProboscisBackOverBar.xml':'ProboscisBackOverBar.xml',
   'init':'ProboscisPostReconf.xml',
   }

