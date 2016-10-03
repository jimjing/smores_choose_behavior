# config.py
from name_map import *

"""
Simulator names:

3   4
2-0-1 -> front
5   6
"""

from numpy import pi

angle = 90
ModuleMap = {'SMORES_0':middle, 'SMORES_1':front, 'SMORES_2':back, 'SMORES_3':back_l,'SMORES_4':front_l, 'SMORES_5':back_r, 'SMORES_6':front_r}
NeutralPositions = {}

disabledDof=['tilt']
speed_scale=0.7
behaviorFiles = {
   'Tank_diff.xml':'Tank_diff.xml',
   }

