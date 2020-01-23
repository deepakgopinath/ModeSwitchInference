# 2017.12.19 17:59:23 CST
#Embedded file name: /home/alex/Desktop/smart_wheelchair/src/limited_control_planning/src_2/nodes.py
import numpy as np
from copy import copy

class node:
    thresh = 0.001

    def __init__(self, x, y, theta, action = 0, distance = 0, priority = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.action = action
        self.distance = distance
        self.priority = priority

    def __lt__(self, other):
        return self.priority < other.priority

    def __repr__(self):
        return 'x : ' + str(self.x) + ', y : ' + str(self.y) + ', theta : ' + str(self.idx_to_angle(self.theta)) + ', action : ' + str(self.action)

    def nextMove(self, action, d):
        if action == 0:
            if d == 2:
                self.distance += 10
            else:
                self.distance += 20
        elif d == 1:
            self.distance += 10
        else:
            self.distance += 20

    def updatePriority(self, xDest, yDest, thetaDest, actionDest):
        self.priority = self.distance + self.estimate(xDest, yDest, thetaDest, actionDest) * 10

    def estimate(self, xDest, yDest, thetaDest, actionDest):
        xd = xDest - self.x
        yd = yDest - self.y
        d = np.sqrt(xd * xd + yd * yd)
        return d

    def get_transition_states(self):
        if self.action == 0:
            pos, neg = self.get_transition_angles()
            s0 = self.new_angle_state(neg)
            s0.action = 0
            s1 = self.new_angle_state(pos)
            s1.action = 0
            s2 = node(self.x, self.y, self.theta, self.action)
            s2.action = 1
            dx = [s0.x, s1.x, s2.x]
            dy = [s0.y, s1.y, s2.y]
            dtheta = [s0.theta, s1.theta, s2.theta]
            daction = [s0.action, s1.action, s2.action]
        else:
            pos, neg = self.get_transition_angles()
            s0 = node(self.x, self.y, self.theta, self.action)
            s0.action = 0
            s1 = self.new_pos_state(self.theta)
            s1.action = 1
            s2 = self.new_pos_state(neg)
            s2.action = 1
            s3 = self.new_pos_state(pos)
            s3.action = 1
            dx = [s0.x,
             s1.x,
             s2.x,
             s3.x]
            dy = [s0.y,
             s1.y,
             s2.y,
             s3.y]
            dtheta = [s0.theta,
             s1.theta,
             s2.theta,
             s3.theta]
            daction = [s0.action,
             s1.action,
             s2.action,
             s3.action]
        return [dx,
         dy,
         dtheta,
         daction]

    def get_transition_angles(self):
        pos = (self.theta + 1) % 8
        neg = (self.theta - 1) % 8
        return (pos, neg)

    def new_pos_state(self, idx):
        ang = self.idx_to_angle(idx)
        dx, dy = (0, 0)
        if np.cos(ang) > self.thresh:
            dx = 1
        elif np.cos(ang) < -self.thresh:
            dx = -1
        if np.sin(ang) > self.thresh:
            dy = 1
        elif np.sin(ang) < -self.thresh:
            dy = -1
        return node(self.x + dx, self.y + dy, idx, self.action)

    def new_angle_state(self, idx):
        return node(self.x, self.y, idx, self.action)

    def idx_to_angle(self, idx):
        return idx * np.pi / 4.0

    def to_str(self):
        return str(self.x) + ',' + str(self.y) + ',' + str(self.theta) + ',' + str(self.action)


class astar_node:
    x = 0
    y = 0
    distance = 0
    priority = 0

    def __init__(self, x, y, distance = 0, priority = 0):
        self.x = x
        self.y = y
        self.distance = distance
        self.priority = priority

    def __lt__(self, other):
        return self.priority < other.priority

    def updatePriority(self, xDest, yDest):
        self.priority = self.distance + self.estimate(xDest, yDest) * 10

    def nextMove(self, dirs, d):
        if dirs == 8 and d % 2 != 0:
            self.distance += 14
        else:
            self.distance += 10

    def estimate(self, xDest, yDest):
        xd = xDest - self.x
        yd = yDest - self.y
        d = np.sqrt(xd * xd + yd * yd)
        return d


class astar_dir_node:

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    def to_str(self):
        return str(self.x) + str(self.y)

    def __repr__(self):
        return '(' + str(self.x) + ',' + str(self.y) + ')'

    def __hash__(self):
        return hash(self.to_str())


class dir_node:

    def __init__(self, x, y, theta, action):
        self.x = x
        self.y = y
        self.theta = theta
        self.action = action

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    def to_str(self):
        return str(self.x) + str(self.y) + str(self.theta) + str(self.action)

    def __repr__(self):
        return '(' + str(self.x) + ',' + str(self.y) + ',' + str(self.theta) + ',' + str(self.action) + ')'

    def __hash__(self):
        return hash(self.to_str())
+++ okay decompyling nodes.pyc 
# decompiled 1 files: 1 okay, 0 failed, 0 verify failed
# 2017.12.19 17:59:23 CST
