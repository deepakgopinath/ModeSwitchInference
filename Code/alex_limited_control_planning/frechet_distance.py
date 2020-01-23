# 2017.12.19 17:58:47 CST
#Embedded file name: /home/alex/Desktop/smart_wheelchair/src/limited_control_planning/src/frechet_distance.py
import numpy as np

def euc_dist(pt1, pt2):
    return np.sqrt(np.power(pt2[0] - pt1[0], 2) + np.power(pt2[1] - pt1[1], 2))


def _c(ca, i, j, P, Q):
    if ca[i, j] > -1:
        return ca[i, j]
    if i == 0 and j == 0:
        ca[i, j] = euc_dist(P[0], Q[0])
    elif i > 0 and j == 0:
        ca[i, j] = max(_c(ca, i - 1, 0, P, Q), euc_dist(P[i], Q[0]))
    elif i == 0 and j > 0:
        ca[i, j] = max(_c(ca, 0, j - 1, P, Q), euc_dist(P[0], Q[j]))
    elif i > 0 and j > 0:
        ca[i, j] = max(min(_c(ca, i - 1, j, P, Q), _c(ca, i - 1, j - 1, P, Q), _c(ca, i, j - 1, P, Q)), euc_dist(P[i], Q[j]))
    else:
        ca[i, j] = float('inf')
    return ca[i, j]


def frechetDist(P, Q):
    ca = np.ones((len(P), len(Q)))
    ca = np.multiply(ca, -1)
    return _c(ca, len(P) - 1, len(Q) - 1, P, Q)
+++ okay decompyling frechet_distance.pyc 
# decompiled 1 files: 1 okay, 0 failed, 0 verify failed
# 2017.12.19 17:58:47 CST
