# 2017.12.19 17:57:46 CST
#Embedded file name: /home/alex/Desktop/current_projects/smart_wheelchair/src/limited_control_planning/src_2/astar_3d_nodes.py
import numpy as np
from copy import copy

class astar_3d_node:
    thresh = 0.001

    def __init__(self, x, y, theta, distance = 0, priority = 0):
        self.x = x
        self.y = y
        self.theta = theta
        self.distance = distance
        self.priority = priority

    def __lt__(self, other):
        return self.priority < other.priority

    def __repr__(self):
        return 'x : ' + str(self.x) + ', y : ' + str(self.y) + ', theta : ' + str(self.idx_to_angle(self.theta))

    def nextMove(self, d):
        if d == 0:
            self.distance += 10
        else:
            self.distance += 1

    def updatePriority(self, xDest, yDest, thetaDest):
        t = self.get_mod_diff(self.theta, thetaDest)
        t = t / float(8.0)
        self.priority = self.distance + t * 10 + self.estimate(xDest, yDest, thetaDest) * 10

    def get_mod_diff(self, ang1, ang2):
        thetaDiff = abs(ang2 - ang1)
        return thetaDiff

    def estimate(self, xDest, yDest, thetaDest):
        xd = xDest - self.x
        yd = yDest - self.y
        d = np.sqrt(xd * xd + yd * yd)
        return d

    def get_transition_states(self):
        s0 = self.new_pos_state(self.theta)
        thetas = range(0, 8)
        thetas.remove(self.theta)
        s1 = self.new_angle_state(thetas[0])
        s2 = self.new_angle_state(thetas[1])
        s3 = self.new_angle_state(thetas[2])
        s4 = self.new_angle_state(thetas[3])
        s5 = self.new_angle_state(thetas[4])
        s6 = self.new_angle_state(thetas[5])
        s7 = self.new_angle_state(thetas[6])
        dx = [s0.x,
         s1.x,
         s2.x,
         s3.x,
         s4.x,
         s5.x,
         s6.x,
         s7.x]
        dy = [s0.y,
         s1.y,
         s2.y,
         s3.y,
         s4.y,
         s5.y,
         s6.y,
         s7.y]
        dtheta = [s0.theta,
         s1.theta,
         s2.theta,
         s3.theta,
         s4.theta,
         s5.theta,
         s6.theta,
         s7.theta]
        return [dx, dy, dtheta]

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
        return astar_3d_node(self.x + dx, self.y + dy, idx)

    def new_angle_state(self, idx):
        return astar_3d_node(self.x, self.y, idx)

    def idx_to_angle(self, idx):
        return idx * np.pi / 4.0

    def to_str(self):
        return str(self.x) + ',' + str(self.y) + ',' + str(self.theta)


class astar_3d_dir_node:

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    def to_str(self):
        return str(self.x) + str(self.y) + str(self.theta)

    def __repr__(self):
        return '(' + str(self.x) + ',' + str(self.y) + ',' + str(self.theta) + ')'

    def __hash__(self):
        return hash(self.to_str())
+++ okay decompyling astar_3d_nodes.pyc 
# decompiled 1 files: 1 okay, 0 failed, 0 verify failed
# 2017.12.19 17:57:46 CST
