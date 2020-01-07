# 2017.12.19 17:59:45 CST
#Embedded file name: /home/alex/Desktop/smart_wheelchair/src/limited_control_planning/src_2/print_map.py
import numpy as npw

def print_map(the_map):
    print 'Map:'
    for y in range(the_map.shape[3]):
        for x in range(the_map.shape[2]):
            xy = the_map[0][0][y][x]
            if xy == 0:
                print '.',
            elif xy == 1:
                print 'O',
            elif xy == 2:
                print 'S',
            elif xy == 3:
                print 'R',
            elif xy == 4:
                print 'F',

        print
+++ okay decompyling print_map.pyc 
# decompiled 1 files: 1 okay, 0 failed, 0 verify failed
# 2017.12.19 17:59:45 CST
