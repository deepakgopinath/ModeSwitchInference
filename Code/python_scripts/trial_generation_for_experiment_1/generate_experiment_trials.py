#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
sys.path.append('../utils_ms')
from utils_ms import SCALE, VIEWPORT_W, VIEWPORT_H, ROBOT_RADIUS, GOAL_RADIUS, PI
from utils_ms import AssistanceType, StartDirection
import numpy as np
import os
import pickle
import collections
import itertools
import argparse
import random
from IPython import embed

VIEWPORT_WS = VIEWPORT_W/SCALE
VIEWPORT_HS = VIEWPORT_H/SCALE
ROBOT_RADIUS_S = ROBOT_RADIUS/SCALE
GOAL_RADIUS_S = GOAL_RADIUS/SCALE

RG_CONFIGS = ['tr', 'tl', 'bl', 'br']
NUM_TURNS = [3]
START_DIRECTIONS = [StartDirection.X, StartDirection.Y]
ASSISTANCE_TYPES = [AssistanceType.Filter, AssistanceType.Corrective, AssistanceType.No_Assistance]

ROBOT_GOAL_CONFIGURATIONS = {'tr':{'robot':[VIEWPORT_WS/4, VIEWPORT_HS/4],     'goal':[3*VIEWPORT_WS/4, 3*VIEWPORT_HS/4]},
                             'tl':{'robot':[3*VIEWPORT_WS/4, VIEWPORT_HS/4],   'goal':[VIEWPORT_WS/4, 3*VIEWPORT_HS/4]},
                             'bl':{'robot':[3*VIEWPORT_WS/4, 3*VIEWPORT_HS/4], 'goal':[VIEWPORT_WS/4, VIEWPORT_HS/4]},
                             'br':{'robot':[VIEWPORT_WS/4, 3*VIEWPORT_HS/4],   'goal':[3*VIEWPORT_WS/4, VIEWPORT_HS/4]}
                            }
START_MODE_DICT = {StartDirection.X: {-1: 't', 1: 'y'},
                   StartDirection.Y: {-1: 'x', 1: 't'}}

START_MODE_OPTIONS = [-1, 1] # if (x,y, theta) is the mode sequence, -1 refers to the mode to the 'left' of the start direction and +1 refers to the mode on the right direction.

def generate_experiment_trials(args):
    num_reps_per_condition = args.num_reps_per_condition
    trial_dir = args.trial_dir
    index = 0

    for rg_config, num_turns, start_direction, assistance_type  in itertools.product(RG_CONFIGS, NUM_TURNS, START_DIRECTIONS, ASSISTANCE_TYPES):
        trial_info_dict = collections.OrderedDict()
        trial_info_dict['env_params'] = collections.OrderedDict()
        trial_info_dict['env_params']['r_to_g_relative_orientation'] = rg_config
        trial_info_dict['env_params']['num_turns'] = num_turns
        trial_info_dict['env_params']['start_direction'] = start_direction
        trial_info_dict['env_params']['assistance_type'] = assistance_type

        trial_info_dict['env_params']['robot_position'] = ROBOT_GOAL_CONFIGURATIONS[rg_config]['robot']
        trial_info_dict['env_params']['robot_orientation'] = 0.0
        trial_info_dict['env_params']['goal_position'] = ROBOT_GOAL_CONFIGURATIONS[rg_config]['goal']
        trial_info_dict['env_params']['goal_orientation'] = PI/2

        start_mode_option = random.choice(START_MODE_OPTIONS)
        trial_info_dict['env_params']['start_mode'] = START_MODE_DICT[start_direction][start_mode_option]
        trial_info_dict['env_params']['location_of_turn'] = random.choice(range(1, num_turns+1))

        for j in range(num_reps_per_condition):
            with open(os.path.join(trial_dir, str(index) + '.pkl'), 'wb') as fp:
                pickle.dump(trial_info_dict, fp)
            index += 1
            print 'Trial Number ', index

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--trial_dir', dest='trial_dir',default=os.path.join(os.getcwd(), 'trial_dir'), help="The directory where trials will be stored are")
    parser.add_argument('--num_reps_per_condition', action='store', type=int, default=3, help="number of repetetions for single combination of conditions ")
    args = parser.parse_args()
    generate_experiment_trials(args)
