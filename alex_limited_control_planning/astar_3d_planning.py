# 2017.12.19 17:57:57 CST
#Embedded file name: /home/alex/Desktop/current_projects/smart_wheelchair/src/limited_control_planning/src_2/astar_3d_planning.py
from heapq import heappush, heappop
from astar_3d_nodes import astar_3d_dir_node, astar_3d_node
import numpy as np

def astar_3d_path_find(the_map, xA, yA, thetaA, xB, yB, thetaB):
    closed_nodes_map = np.zeros(shape=the_map.shape)
    open_nodes_map = np.zeros(shape=the_map.shape)
    dir_map = {}
    explored_nodes_count = 0
    pq = [[], []]
    pqi = 0
    n0 = astar_3d_node(xA, yA, thetaA)
    n0.updatePriority(xB, yB, thetaB)
    heappush(pq[pqi], n0)
    open_nodes_map[thetaA][yA][xA] = n0.priority
    while len(pq[pqi]) > 0:
        n1 = pq[pqi][0]
        n0 = astar_3d_node(n1.x, n1.y, n1.theta, n1.distance, n1.priority)
        x = n0.x
        y = n0.y
        theta = n0.theta
        heappop(pq[pqi])
        open_nodes_map[theta][y][x] = 0
        closed_nodes_map[theta][y][x] = 1
        if x == xB and y == yB and theta == thetaB:
            path = []
            while not (x == xA and y == yA and theta == thetaA):
                td_node = astar_3d_dir_node(x, y, theta)
                parent = dir_map[td_node]
                path.append(parent)
                x = parent.x
                y = parent.y
                theta = parent.theta

            return (path, explored_nodes_count)
        dx, dy, dtheta = n0.get_transition_states()
        for i in range(len(dx)):
            xdx = dx[i]
            ydy = dy[i]
            thetadtheta = dtheta[i]
            if not (xdx < 0 or xdx > the_map.shape[2] - 1 or ydy < 0 or ydy > the_map.shape[1] - 1 or the_map[thetadtheta][ydy][xdx] == 1 or closed_nodes_map[thetadtheta][ydy][xdx] == 1):
                m0 = astar_3d_node(xdx, ydy, thetadtheta, n0.distance, n0.priority)
                m0.nextMove(i)
                m0.updatePriority(xB, yB, thetaB)
                if open_nodes_map[thetadtheta][ydy][xdx] == 0:
                    explored_nodes_count += 1
                    open_nodes_map[thetadtheta][ydy][xdx] = m0.priority
                    heappush(pq[pqi], m0)
                    md_node = astar_3d_dir_node(m0.x, m0.y, m0.theta)
                    nd_node = astar_3d_dir_node(n0.x, n0.y, n0.theta)
                    dir_map[md_node] = nd_node
                elif open_nodes_map[thetadtheta][ydy][xdx] > m0.priority:
                    open_nodes_map[thetadtheta][ydy][xdx] = m0.priority
                    md_node = astar_3d_dir_node(m0.x, m0.y, m0.theta)
                    nd_node = astar_3d_dir_node(n0.x, n0.y, n0.theta)
                    dir_map[md_node] = nd_node
                    if len(pq[pqi]) > 0:
                        while not (pq[pqi][0].x == xdx and pq[pqi][0].y == ydy):
                            heappush(pq[1 - pqi], pq[pqi][0])
                            heappop(pq[pqi])

                        heappop(pq[pqi])
                        if len(pq[pqi]) > len(pq[1 - pqi]):
                            pqi = 1 - pqi
                        while len(pq[pqi]) > 0:
                            heappush(pq[1 - pqi], pq[pqi][0])
                            heappop(pq[pqi])

                        pqi = 1 - pqi
                        heappush(pq[pqi], m0)

    return ('', explored_nodes_count)
+++ okay decompyling astar_3d_planning.pyc 
# decompiled 1 files: 1 okay, 0 failed, 0 verify failed
# 2017.12.19 17:57:57 CST
