#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import SCALE, VIEWPORT_W, VIEWPORT_H, ROBOT_RADIUS, GOAL_RADIUS

import numpy as np
import math
import os
import pickle
from collections import OrderedDict

from IPython import embed

QUADRANT_BOUNDS = OrderedDict()
VIEWPORT_WS = VIEWPORT_W/SCALE
VIEWPORT_HS = VIEWPORT_H/SCALE
ROBOT_RADIUS_S = ROBOT_RADIUS/SCALE
GOAL_RADIUS_S = GOAL_RADIUS/SCALE
R_TO_G_CONFIGS = {'tr':{'r':'3', 'g': '1'},
                  'tl':{'r':'4', 'g': '2'},
                  'br':{'r':'2', 'g': '4'},
                  'bl':{'r':'1', 'g': '3'}}

def create_bounds_dict():
    q_keys = [1,2,3,4]
    dimensions = ['x', 'y']
    bounds = ['min', 'max']
    for q_key in q_keys:
        QUADRANT_BOUNDS[str(q_key)] = OrderedDict()
        for d in dimensions:
            QUADRANT_BOUNDS[str(q_key)][d] = OrderedDict()
            for b in bounds:
                QUADRANT_BOUNDS[str(q_key)][d][b] = None


def initialize_bounds():
    #initialize bounds for 'tr'
    QUADRANT_BOUNDS['1']['x']['min'] = (2*VIEWPORT_WS)/3 + ROBOT_RADIUS_S
    QUADRANT_BOUNDS['1']['x']['max'] = VIEWPORT_WS - ROBOT_RADIUS_S
    QUADRANT_BOUNDS['1']['y']['min'] = (2*VIEWPORT_HS)/3 + ROBOT_RADIUS_S
    QUADRANT_BOUNDS['1']['y']['max'] = VIEWPORT_HS - ROBOT_RADIUS_S

    #initalize bounds for 'tl'
    QUADRANT_BOUNDS['2']['x']['min'] = ROBOT_RADIUS_S
    QUADRANT_BOUNDS['2']['x']['max'] = VIEWPORT_WS/3 - ROBOT_RADIUS_S
    QUADRANT_BOUNDS['2']['y']['min'] = (2*VIEWPORT_HS)/3 + ROBOT_RADIUS_S
    QUADRANT_BOUNDS['2']['y']['max'] = VIEWPORT_HS - ROBOT_RADIUS_S

    #initialize_bounds for 'bl'
    QUADRANT_BOUNDS['3']['x']['min'] = ROBOT_RADIUS_S
    QUADRANT_BOUNDS['3']['x']['max'] = VIEWPORT_WS/3 - ROBOT_RADIUS_S
    QUADRANT_BOUNDS['3']['y']['min'] = ROBOT_RADIUS_S
    QUADRANT_BOUNDS['3']['y']['max'] = VIEWPORT_HS/3 - ROBOT_RADIUS_S

    #initialize_bounds for 'br'
    QUADRANT_BOUNDS['4']['x']['min'] = (2*VIEWPORT_WS)/3 + ROBOT_RADIUS_S
    QUADRANT_BOUNDS['4']['x']['max'] = VIEWPORT_WS - ROBOT_RADIUS_S
    QUADRANT_BOUNDS['4']['y']['min'] = ROBOT_RADIUS_S
    QUADRANT_BOUNDS['4']['y']['max'] = VIEWPORT_HS/3 - ROBOT_RADIUS_S

def create_r_to_g_configurations(num_trials):
    r_to_g_config_list = []
    for key in R_TO_G_CONFIGS.keys():
        r_to_g_config_list.extend([key]*(num_trials//4))

    return r_to_g_config_list

def generate_r_and_g_positions(r_to_g_config_list):
    num_trials = len(r_to_g_config_list)
    robot_positions = np.zeros((num_trials,2))
    goal_positions = np.zeros((num_trials, 2))
    for i, rgc in enumerate(r_to_g_config_list):
        rq = R_TO_G_CONFIGS[rgc]['r']
        gq = R_TO_G_CONFIGS[rgc]['g']
        rx = QUADRANT_BOUNDS[rq]['x']['min'] + np.random.random()*(QUADRANT_BOUNDS[rq]['x']['max']-QUADRANT_BOUNDS[rq]['x']['min'])
        ry = QUADRANT_BOUNDS[rq]['y']['min'] + np.random.random()*(QUADRANT_BOUNDS[rq]['y']['max']-QUADRANT_BOUNDS[rq]['y']['min'])
        gx = QUADRANT_BOUNDS[gq]['x']['min'] + np.random.random()*(QUADRANT_BOUNDS[gq]['x']['max']-QUADRANT_BOUNDS[gq]['x']['min'])
        gy = QUADRANT_BOUNDS[gq]['y']['min'] + np.random.random()*(QUADRANT_BOUNDS[gq]['y']['max']-QUADRANT_BOUNDS[gq]['y']['min'])
        robot_positions[i] = (rx, ry)
        goal_positions[i] = (gx, gy)

    return robot_positions, goal_positions

def generate_r_and_g_orientations(num_trials):
    pass

def generate_trials(num_trials=40):
    create_bounds_dict()
    initialize_bounds()
    r_to_g_config_list = create_r_to_g_configurations(num_trials)
    robot_init_positions, goal_positions = generate_r_and_g_positions(r_to_g_config_list)
    robot_init_orientations, goal_orientations = generate_r_and_g_orientations()
    embed()

if __name__ == '__main__':
    generate_trials()
