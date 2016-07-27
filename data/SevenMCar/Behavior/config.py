# config.py

"""
Simulator names:

4   6
1-0-2
3   5
"""

from numpy import pi

angle = 90
ModuleMap = {'SMORES_0':13, 'SMORES_1':12, 'SMORES_2':14, 'SMORES_3':3,
             'SMORES_4':16, 'SMORES_5':9,'SMORES_6':8,}
NeutralPositions = {}

disabledDof=['tilt']
speed_scale=0.7
behaviorFiles = {
   'SevenMCarDiff.xml':'SevenMCarDiff.xml',
   'SevenMCarPreReconf.xml':'SevenMCarPreReconf.xml',
   }

