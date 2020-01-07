# 2017.12.19 17:59:01 CST
#Embedded file name: /home/alex/Desktop/smart_wheelchair/src/limited_control_planning/src/load_map.py
import numpy as np

def load_map():
    actions = ['rotation', 'translation']
    num_actions = len(actions)
    map_size = 400
    num_angs = 8
    the_map = np.zeros(shape=(num_actions,
     num_angs,
     map_size,
     map_size))
    for state_idx in range(the_map.shape[0]):
        for ang_idx in range(the_map.shape[1]):
            for x in range(the_map.shape[2] / 8, the_map.shape[2] * 7 / 8):
                the_map[state_idx][ang_idx][the_map.shape[3] / 2][x] = 1

            for y in range(the_map.shape[3] / 8, the_map.shape[3] * 7 / 8):
                the_map[state_idx][ang_idx][y][the_map.shape[2] / 2] = 1

    xA = the_map.shape[2] - 1
    yA = the_map.shape[3] / 2 + 1
    thetaA = 0
    actionA = 0
    xB = 0
    yB = the_map.shape[3] / 2 - 1
    thetaB = 0
    actionB = 0
    return [the_map,
     xA,
     yA,
     thetaA,
     actionA,
     xB,
     yB,
     thetaB,
     actionB]


def load_saved_map():
    actions = ['rotation', 'translation']
    num_actions = len(actions)
    map_size = 400
    num_angs = 8
    the_map = np.load('/home/alex/Desktop/smart_wheelchair/src/limited_control_planning/map.npy')
    xA = np.load('/home/alex/Desktop/smart_wheelchair/src/limited_control_planning/x.npy')
    yA = np.load('/home/alex/Desktop/smart_wheelchair/src/limited_control_planning/y.npy')
    thetaA = 0
    actionA = 0
    xB = 100
    yB = 100
    thetaB = 0
    actionB = 0
    return [the_map,
     xA,
     yA,
     thetaA,
     actionA,
     xB,
     yB,
     thetaB,
     actionB]
+++ okay decompyling load_map.pyc 
# decompiled 1 files: 1 okay, 0 failed, 0 verify failed
# 2017.12.19 17:59:01 CST
