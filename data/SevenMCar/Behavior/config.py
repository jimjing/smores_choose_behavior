# config.py

"""
Simulator names:

4   6
1-0-2
3   5
"""

from numpy import pi

angle = 90
ModuleMap = {'SMORES_0':16, 'SMORES_1':2, 'SMORES_2':18, 'SMORES_3':12,
             'SMORES_4':9, 'SMORES_5':14,'SMORES_6':13,}
NeutralPositions = {}

disabledDof=['tilt']
speed_scale=0.7
behaviorFiles = {
   'SevenMCarDiff.xml':'SevenMCarDiff.xml',
   'SevenMCarPreReconf.xml':'SevenMCarPreReconf.xml',
   }

