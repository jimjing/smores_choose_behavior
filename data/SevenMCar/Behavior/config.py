# config.py

"""
Simulator names:

4   6
1-0-2
3   5
"""

from numpy import pi

angle = 90
ModuleMap = {'SMORES_0':16, 'SMORES_1':18, 'SMORES_2':2, 'SMORES_3':14,
             'SMORES_4':13, 'SMORES_5':12,'SMORES_6':9,}
NeutralPositions = {}

disabledDof=['tilt']
speed_scale=0.7
behaviorFiles = {
   'SevenMCarDiff.xml':'SevenMCarDiff.xml',
   'SevenMCarPreReconf.xml':'SevenMCarPreReconf.xml',
   }

