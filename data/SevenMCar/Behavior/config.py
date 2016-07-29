# config.py
import sys
sys.path.append('../../')
import name_map

"""
Simulator names:

4   6
1-0-2
3   5
"""

from numpy import pi

angle = 90
#ModuleMap = {'SMORES_0':16, 'SMORES_1':18, 'SMORES_2':2, 'SMORES_3':14,
#             'SMORES_4':13, 'SMORES_5':12,'SMORES_6':9,}
ModuleMap = {'SMORES_0':middle, 'SMORES_1':back_middle, 'SMORES_2':front_middle, 'SMORES_3':back_right,
             'SMORES_4':back_left, 'SMORES_5':front_right,'SMORES_6':front_left,}
NeutralPositions = {}

disabledDof=[]
speed_scale=0.7
behaviorFiles = {
   'SevenMCarDiff.xml':'SevenMCarDiff.xml',
   'SevenMCarPreReconf.xml':'SevenMCarPreReconf.xml',
   }

