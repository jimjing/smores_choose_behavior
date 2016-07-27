# config.py

"""
Simulator names:

4
1-0-2-5-6
3
"""

from numpy import pi

angle = 90
ModuleMap = {'SMORES_0':14, 'SMORES_1':4, 'SMORES_2':12, 'SMORES_3':12,
             'SMORES_4':12, 'SMORES_5':12,'SMORES_6':12,}
NeutralPositions = {}

disabledDof=[]
speed_scale=0.7
behaviorFiles = {
   'ProboscisOverBar.xml':'ProboscisOverBar.xml',
   'ProboscisPickup.xml':'ProboscisPickup.xml',
   'ProboscisBackOverBar.xml':'ProboscisBackOverBar.xml',
   }

