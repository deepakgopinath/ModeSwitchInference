#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import collections
import numpy as np
import sys
sys.path.append('../utils')
from utils import VIEWPORT_H, VIEWPORT_W, SCALE, ROBOT_RADIUS, StartDirection, PI
import itertools
import random
from IPython import embed

NUM_TURNS = 3
NUM_LOCATIONS = NUM_TURNS + 2
VIEWPORT_WS = VIEWPORT_W/SCALE
VIEWPORT_HS = VIEWPORT_H/SCALE
ROBOT_RADIUS_S = ROBOT_RADIUS/SCALE

QUADRANT_BOUNDS = collections.OrderedDict()
WAYPOINTS = np.zeros((NUM_LOCATIONS, 2))
LOCATION_OF_TURN = random.choice(range(1, NUM_LOCATIONS-1))
R_TO_G_CONFIGS = {'tr':{'r':'3', 'g': '1'},
                  'tl':{'r':'4', 'g': '2'},
                  'br':{'r':'2', 'g': '4'},
                  'bl':{'r':'1', 'g': '3'}}


MODES = ['x', 'y', 't']
LOCATIONS = ['p' + str(i) for i in range(NUM_LOCATIONS)]
ORIENTATIONS = [0, PI/2]
STATES = [s for s in itertools.product(LOCATIONS, ORIENTATIONS, MODES)]
ACTIONS = ['hp', 'hs', 'sp', 'ss']
STATE_TRANSITION_MODEL = collections.OrderedDict()
MODE_SWITCH_TRANSITION = {'x': {'hp': 'y', 'hs': 't', 'sp': 'x', 'ss': 'x'},
						  'y': {'hp': 't', 'hs': 'x', 'sp': 'y', 'ss': 'y'},
						  't': {'hp': 'x', 'hs': 'y', 'sp': 't', 'ss': 't'}}
MODES_MOTION_ALLOWED = collections.OrderedDict()

TRANSITION_FOR_ACTION =   {'tr': {'sp': {'x': 'next', 'y': 'next', 't': 'next'}, 'ss': {'x': 'prev', 'y': 'prev', 't': 'prev'}},
					   'tl': {'sp': {'x': 'prev', 'y': 'next', 't': 'next'}, 'ss': {'x': 'next', 'y': 'prev', 't': 'prev'}},
					   'br': {'sp': {'x': 'next', 'y': 'prev', 't': 'next'}, 'ss': {'x': 'prev', 'y': 'next', 't': 'prev'}},
					   'bl': {'sp': {'x': 'prev', 'y': 'prev', 't': 'next'}, 'ss': {'x': 'next', 'y': 'next', 't': 'prev'}}			
						}

ACTION_TO_COMMAND = collections.OrderedDict({'move_p': 'sp', 'move_n':'ss', 'mode_r':'hp', 'mode_l': 'hs'})
COMMAND_TO_ACTION = collections.OrderedDict({v:k for k, v in ACTION_TO_COMMAND.items()})

def create_state_transition_model():
	for s in STATES:
		STATE_TRANSITION_MODEL[s] = collections.OrderedDict()
		for a in ACTIONS:
			STATE_TRANSITION_MODEL[s][a] = None

def init_state_transition_model(rgc):
	for s in STATE_TRANSITION_MODEL.keys():
		for a in STATE_TRANSITION_MODEL[s].keys():
			# print(a)
			if a == 'hp' or a == 'hs':
				STATE_TRANSITION_MODEL[s][a] = (s[0], s[1], MODE_SWITCH_TRANSITION[s[2]][a]) #generate new state
			if a == 'sp' or a == 'ss':
				allowed_modes_for_motion = MODES_MOTION_ALLOWED[s[0]]
				# print(allowed_modes_for_motion)
				for m in allowed_modes_for_motion:
					if m == s[2]: #make sure that the allowed mode matches the mode in the state s. If it doens't no motion will happen
						if m != 't':
							if TRANSITION_FOR_ACTION[rgc][a][m] == 'next':
								new_loc = LOCATIONS[min(LOCATIONS.index(s[0]) + 1, len(LOCATIONS)-1 )]
							elif TRANSITION_FOR_ACTION[rgc][a][m] == 'prev':
								new_loc = LOCATIONS[max(LOCATIONS.index(s[0]) - 1, 0 )]
							STATE_TRANSITION_MODEL[s][a] = (new_loc, s[1], s[2])
						elif m == 't':
							if TRANSITION_FOR_ACTION[rgc][a][m] == 'next':
								new_theta = min(PI/2, s[1] + PI/2)
							elif TRANSITION_FOR_ACTION[rgc][a][m] == 'prev':
								new_theta = max(0, s[1] - PI/2)

							STATE_TRANSITION_MODEL[s][a] = (s[0], new_theta, s[2])
							
					else:
						STATE_TRANSITION_MODEL[s][a] = (s[0], s[1], s[2])
			
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

def generate_waypoints(start_direction):
	if NUM_TURNS % 2 == 0:
		divisor = NUM_TURNS/2 + 1
		if start_direction == StartDirection.X:
			x_inc = (WAYPOINTS[-1][0] - WAYPOINTS[0][0])/divisor
			y_inc = (WAYPOINTS[-1][1] - WAYPOINTS[0][1])/(divisor - 1)
		elif start_direction == StartDirection.Y:
			x_inc = (WAYPOINTS[-1][0] - WAYPOINTS[0][0])/(divisor-1)
			y_inc = (WAYPOINTS[-1][1] - WAYPOINTS[0][1])/divisor
	else:
		divisor = (NUM_TURNS + 1)/2
		x_inc = (WAYPOINTS[-1][0] - WAYPOINTS[0][0])/divisor
		y_inc = (WAYPOINTS[-1][1] - WAYPOINTS[0][1])/divisor

	if start_direction == StartDirection.X:
		for i in range(1, NUM_TURNS+1):
			if i%2 == 0:
				WAYPOINTS[i][0] = WAYPOINTS[i-1][0]
				WAYPOINTS[i][1] = WAYPOINTS[i-1][1] + y_inc
			else:
				WAYPOINTS[i][0] = WAYPOINTS[i-1][0] + x_inc
				WAYPOINTS[i][1] = WAYPOINTS[i-1][1]
	elif start_direction == StartDirection.Y:
		for i in range(1, NUM_TURNS+1):
			if i%2 == 0:
				WAYPOINTS[i][0] = WAYPOINTS[i-1][0] + x_inc
				WAYPOINTS[i][1] = WAYPOINTS[i-1][1]
			else:
				WAYPOINTS[i][0] = WAYPOINTS[i-1][0]
				WAYPOINTS[i][1] = WAYPOINTS[i-1][1] + y_inc

def generate_start_and_end_points(rgc):
	rq = R_TO_G_CONFIGS[rgc]['r']
	gq = R_TO_G_CONFIGS[rgc]['g']
	rx = QUADRANT_BOUNDS[rq]['x']['min'] + np.random.random()*(QUADRANT_BOUNDS[rq]['x']['max']-QUADRANT_BOUNDS[rq]['x']['min'])
	ry = QUADRANT_BOUNDS[rq]['y']['min'] + np.random.random()*(QUADRANT_BOUNDS[rq]['y']['max']-QUADRANT_BOUNDS[rq]['y']['min'])
	gx = QUADRANT_BOUNDS[gq]['x']['min'] + np.random.random()*(QUADRANT_BOUNDS[gq]['x']['max']-QUADRANT_BOUNDS[gq]['x']['min'])
	gy = QUADRANT_BOUNDS[gq]['y']['min'] + np.random.random()*(QUADRANT_BOUNDS[gq]['y']['max']-QUADRANT_BOUNDS[gq]['y']['min'])
	robot_position = (rx, ry)
	goal_position = (gx, gy)
	robot_init_orientation = np.random.random()*(2*PI)
	goal_orientation = robot_init_orientation + PI/2
	return robot_position, goal_position, robot_init_orientation, goal_orientation

def init_modes_in_which_motion_allowed_dict():
	for i, s in enumerate(LOCATIONS[:-1]):
		if i % 2 == 0:
			MODES_MOTION_ALLOWED[s] = ['x'] #Make this a function of start direction
		else:
			MODES_MOTION_ALLOWED[s] = ['y']
		if i == LOCATION_OF_TURN:
			MODES_MOTION_ALLOWED[s].append('t')
			
	MODES_MOTION_ALLOWED[LOCATIONS[-1]] = []

def main():
	r_to_g_config = 'tr'
	start_direction = StartDirection.X
	start_mode = 'y'

	create_bounds_dict()
	initialize_bounds()
	
	robot_position, goal_position, robot_init_orientation, goal_orientation = generate_start_and_end_points(r_to_g_config)
	WAYPOINTS[0] = robot_position
	WAYPOINTS[-1] = goal_position
	generate_waypoints(start_direction)
	init_modes_in_which_motion_allowed_dict()
	create_state_transition_model()
	init_state_transition_model(r_to_g_config)
	embed()
	

if __name__ == '__main__':
	main()