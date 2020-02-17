#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
sys.path.append('../utils_ms')
from utils_ms import SCALE, VIEWPORT_W, VIEWPORT_H, ROBOT_RADIUS, GOAL_RADIUS, PI
from utils_ms import AssistanceType, StartDirection
import numpy as np
import math
import argparse
import os
import random
import pickle
import collections
import itertools
from IPython import embed


QUADRANT_BOUNDS = collections.OrderedDict()
VIEWPORT_WS = VIEWPORT_W/SCALE
VIEWPORT_HS = VIEWPORT_H/SCALE
ROBOT_RADIUS_S = ROBOT_RADIUS/SCALE
GOAL_RADIUS_S = GOAL_RADIUS/SCALE
R_TO_G_CONFIGS = {'tr':{'r':'3', 'g': '1'},
                  'tl':{'r':'4', 'g': '2'},
                  'br':{'r':'2', 'g': '4'},
                  'bl':{'r':'1', 'g': '3'}}

START_MODE_DICT = {'x': {-1: 't', 1: 'y'},
              'y': {-1: 'x', 1: 't'},
              't': {-1: 'y', 1: 'x'}}

#experiment params
RG_CONFIGS = ['tr', 'tl', 'bl', 'br']
NUM_TURNS = [2, 3] #or [1,3]
START_DIMENSION = ['x', 'y']
START_MODE = [-1, 1] # if (x,y, theta) is the mode sequence, -1 refers to the mode to the 'left' of the start direction and +1 refers to the mode on the right direction.
ASSISTANCE_TYPE = [Assistance.Filter, Assistance.Corrective]

R_TO_G_ORIENT_DIFF = PI/2

def create_bounds_dict():
    q_keys = [1,2,3,4]
    dimensions = ['x', 'y']
    bounds = ['min', 'max']
    for q_key in q_keys:
        QUADRANT_BOUNDS[str(q_key)] = collections.OrderedDict()
        for d in dimensions:
            QUADRANT_BOUNDS[str(q_key)][d] = collections.OrderedDict()
            for b in bounds:
                QUADRANT_BOUNDS[str(q_key)][d][b] = None

def initialize_bounds():
    #initialize bounds for 'tr'
    QUADRANT_BOUNDS['1']['x']['min'] = (2*VIEWPORT_WS)/3 + ROBOT_RADIUS_S
    QUADRANT_BOUNDS['1']['x']['max'] = VIEWPORT_WS - ROBOT_RADIUS_S
    QUADRANT_BOUNDS['1']['y']['min'] = (2*VIEWPORT_HS)/3 + ROBOT_RADIUS_S
    QUADRANT_BOUNDS['1']['y']['max'] = VIEWPORT_HS - 3*ROBOT_RADIUS_S#leave some space on the top right corner for the mode display

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

def generate_r_and_g_positions(trial_combinations):
    num_trials = len(trial_combinations)
    robot_positions = np.zeros((num_trials,2))
    goal_positions = np.zeros((num_trials, 2))
    for i, tc in enumerate(trial_combinations):
        rgc = tc[0]
        rq = R_TO_G_CONFIGS[rgc]['r']
        gq = R_TO_G_CONFIGS[rgc]['g']
        rx = QUADRANT_BOUNDS[rq]['x']['min'] + np.random.random()*(QUADRANT_BOUNDS[rq]['x']['max']-QUADRANT_BOUNDS[rq]['x']['min'])
        ry = QUADRANT_BOUNDS[rq]['y']['min'] + np.random.random()*(QUADRANT_BOUNDS[rq]['y']['max']-QUADRANT_BOUNDS[rq]['y']['min'])
        gx = QUADRANT_BOUNDS[gq]['x']['min'] + np.random.random()*(QUADRANT_BOUNDS[gq]['x']['max']-QUADRANT_BOUNDS[gq]['x']['min'])
        gy = QUADRANT_BOUNDS[gq]['y']['min'] + np.random.random()*(QUADRANT_BOUNDS[gq]['y']['max']-QUADRANT_BOUNDS[gq]['y']['min'])
        robot_positions[i] = (rx, ry)
        goal_positions[i] = (gx, gy)

    return robot_positions, goal_positions

def generate_r_and_g_orientations(trial_combinations):
    num_trials = len(trial_combinations)
    robot_init_orientations = np.zeros((num_trials, 1))
    goal_orientations = np.zeros((num_trials, 1))
    for i in range(num_trials):
        robot_init_orientations[i] = np.random.random()*(2*PI)
        if np.random.random() < 0.5:
            goal_orientations[i] = robot_init_orientations[i] + R_TO_G_ORIENT_DIFF
        else:
            goal_orientations[i] = robot_init_orientations[i] - R_TO_G_ORIENT_DIFF

    return robot_init_orientations, goal_orientations

def generate_trials(args):

    num_reps_per_condition = args.num_reps_per_condition
    trial_dir = args.trial_dir
    if not os.path.exists(trial_dir):
        os.makedirs(trial_dir)

    trial_combinations = []
    #generate full factorial design
    for combination in itertools.product(RG_CONFIGS, NUM_TURNS, START_DIMENSION, START_MODE):
        trial_combinations.append(combination)

    trial_combinations = trial_combinations

    create_bounds_dict()
    initialize_bounds()
    robot_init_positions, goal_positions = generate_r_and_g_positions(trial_combinations)
    robot_init_orientations, goal_orientations = generate_r_and_g_orientations(trial_combinations)

    #repeat
    robot_init_positions = np.tile(robot_init_positions, (num_reps_per_condition, 1))
    goal_positions = np.tile(goal_positions, (num_reps_per_condition, 1))
    robot_init_orientations = np.tile(robot_init_orientations, (num_reps_per_condition, 1))
    goal_orientations = np.tile(goal_orientations, (num_reps_per_condition, 1))
    trial_combinations = trial_combinations * num_reps_per_condition

    total_num_trials = num_reps_per_condition * len(RG_CONFIGS) *len(NUM_TURNS) * len(START_DIMENSION) *len(START_MODE)

    assert len(trial_combinations) == robot_init_positions.shape[0] == goal_positions.shape[0] == robot_init_orientations.shape[0] == goal_orientations.shape[0] == total_num_trials
    assert trial_dir is not None

    for i in range(total_num_trials):
        trial_info_dict = collections.OrderedDict()
        trial_info_dict['env_params'] = collections.OrderedDict()
        trial_info_dict['env_params']['robot_position'] = robot_init_positions[i]
        trial_info_dict['env_params']['goal_position'] = goal_positions[i]
        trial_info_dict['env_params']['robot_orientation'] = robot_init_orientations[i]
        trial_info_dict['env_params']['goal_orientation'] = goal_orientations[i]

        trial_info_dict['env_params']['r_to_g_relative_orientation'] = trial_combinations[0]
        trial_info_dict['env_params']['num_turns'] = trial_combinations[i][1]
        trial_info_dict['env_params']['start_direction'] = trial_combinations[i][2]

        start_direction = trial_combinations[i][2]
        start_mode_option = trial_combinations[i][3]
        trial_info_dict['env_params']['start_mode'] = START_MODE_DICT[start_direction][start_mode_option]
        if trial_info_dict['env_params']['start_mode'] == 't':
            location_of_turn = 0
        else:
            location_of_turn = random.choice(range(1, trial_info_dict['env_params']['num_turns'] + 1)) #can't be the first (0) or the last location for turning

        trial_info_dict['env_params']['location_of_turn'] = location_of_turn

        with open(os.path.join(trial_dir, str(i) + '.pkl'), 'wb') as fp:
            pickle.dump(trial_info_dict, fp)

        print 'Trial number ', i


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--trial_dir', dest='trial_dir',default=os.path.join(os.getcwd(), 'trial_dir'), help="The directory where trials will be stored are")
    parser.add_argument('--num_reps_per_condition', action='store', type=int, default=2, help="number of repetetions for single combination of conditions ")
    args = parser.parse_args()
    generate_trials(args)
