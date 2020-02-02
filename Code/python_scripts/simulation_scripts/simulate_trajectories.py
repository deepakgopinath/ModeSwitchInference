#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import collections
import numpy as np
import sys
sys.path.append('../utils')
from utils import VIEWPORT_H, VIEWPORT_W, SCALE, ROBOT_RADIUS, StartDirection, PI, AssistanceType
import itertools
import random
from IPython import embed

np.random.seed(0) #seed for reproducibility

NUM_TURNS = 3 #number of turns in the path
NUM_LOCATIONS = NUM_TURNS + 2 #total number of 'pitstops' = turns+start+end point

#params for generating robot positions. Not used
VIEWPORT_WS = VIEWPORT_W/SCALE
VIEWPORT_HS = VIEWPORT_H/SCALE
ROBOT_RADIUS_S = ROBOT_RADIUS/SCALE
QUADRANT_BOUNDS = collections.OrderedDict()

#array to hold the WAYPOINTS. continuous valued positions
WAYPOINTS = np.zeros((NUM_LOCATIONS, 2))

#The corner at which the robot needs to be turned
LOCATION_OF_TURN = random.choice(range(1, NUM_LOCATIONS-1))

#depending on the relative configuration of the goal with respect to the robot, robot and g will be in different quadrants
R_TO_G_CONFIGS = {'tr':{'r':'3', 'g': '1'},
                  'tl':{'r':'4', 'g': '2'},
                  'br':{'r':'2', 'g': '4'},
                  'bl':{'r':'1', 'g': '3'}}

#1D modes for SNP
MODES = ['x', 'y', 't']
#symbols for different locations. 
LOCATIONS = ['p' + str(i) for i in range(NUM_LOCATIONS)]
#two distinct orientations. Initially robot is at 0, after turning the robot will be at PI/2
ORIENTATIONS = [0, PI/2]
#Generate list of states. State = (location, orientation, mode)
STATES = [s for s in itertools.product(LOCATIONS, ORIENTATIONS, MODES)]
#low level commands issued by the snp interface. hp = hard puff, hs= hard sip, sp = soft puff, ss = soft sip. Also the domain for ui and um
LOW_LEVEL_COMMANDS = ['hp', 'hs', 'sp', 'ss']
#high level actions, move_p = move in positive direction, move_n = move in negative direction, mode_r = switch mode to right, mode_l = switch mode to left. positive and negative is conditioned on mode
HIGH_LEVEL_ACTIONS = ['move_p', 'move_n', 'mode_r', 'mode_l']

#empty dictionary to hold the state transition mode
STATE_TRANSITION_MODEL = collections.OrderedDict()

#transition function for mode switches. 
MODE_SWITCH_TRANSITION = {'x': {'hp': 'y', 'hs': 't', 'sp': 'x', 'ss': 'x'},
						  'y': {'hp': 't', 'hs': 'x', 'sp': 'y', 'ss': 'y'},
						  't': {'hp': 'x', 'hs': 'y', 'sp': 't', 'ss': 't'}}

#empty dictionary to store information regarding modes that allow motion. Essentially this is a way to do bound checks in a path
MODES_MOTION_ALLOWED = collections.OrderedDict()

#Depending on the configuration of the initial robot position and goal position, the motion commands will result in either moving towards the
#next location or the previous location
TRANSITION_FOR_ACTION =   {'tr': {'sp': {'x': 'next', 'y': 'next', 't': 'next'}, 'ss': {'x': 'prev', 'y': 'prev', 't': 'prev'}},
					   'tl': {'sp': {'x': 'prev', 'y': 'next', 't': 'next'}, 'ss': {'x': 'next', 'y': 'prev', 't': 'prev'}},
					   'br': {'sp': {'x': 'next', 'y': 'prev', 't': 'next'}, 'ss': {'x': 'prev', 'y': 'next', 't': 'prev'}},
					   'bl': {'sp': {'x': 'prev', 'y': 'prev', 't': 'next'}, 'ss': {'x': 'next', 'y': 'next', 't': 'prev'}}			
						}

#true mapping of a to u
TRUE_ACTION_TO_COMMAND = collections.OrderedDict({'move_p': 'sp', 'move_n':'ss', 'mode_r':'hp', 'mode_l': 'hs'})
#true inverse mapping of u to a
TRUE_COMMAND_TO_ACTION = collections.OrderedDict({v:k for k, v in TRUE_ACTION_TO_COMMAND.items()})
#dictionary to hold the optimal next state for a given state. Essentially a one-step optimal plan
OPTIMAL_NEXT_STATE_DICT = collections.OrderedDict()
#dictionary to hold the action to go the optimal next state
OPTIMAL_ACTION_DICT = collections.OrderedDict()

#p(ui|a)
P_UI_GIVEN_A = collections.OrderedDict() 
UI_GIVEN_A_NOISE = 0.1 #Lower the number, lower the error. Between 0 and 1. If 0, the p(ui|a) is delta and same as the true mapping

#p(um|ui)
P_UM_GIVEN_UI = collections.OrderedDict()
UM_GIVEN_UI_NOISE = 0.9 #Lower the number, lower the error. Between 0 and 1. If 0, no difference between ui and um

#p(ui|um). dictionary to hold inferred ui
P_UI_GIVEN_UM = collections.OrderedDict()
#entropy threshold for assistance. between 0 and 1
ENTROPY_THRESHOLD = 0.5

#Assistance Type. Choice between Filter and Corrective. TODO. Maybe load it from file
ASSISTANCE_TYPE = AssistanceType.Corrective

#Boolean that decides whether the simulation is fully 'manual' vs. assisted
IS_ASSISTANCE = False

def create_state_transition_model():
	for s in STATES:
		STATE_TRANSITION_MODEL[s] = collections.OrderedDict()
		for u in LOW_LEVEL_COMMANDS:
			STATE_TRANSITION_MODEL[s][u] = None

def init_state_transition_model(rgc):
	for s in STATE_TRANSITION_MODEL.keys():
		for u in STATE_TRANSITION_MODEL[s].keys():
			if u == 'hp' or u == 'hs':
				STATE_TRANSITION_MODEL[s][u] = (s[0], s[1], MODE_SWITCH_TRANSITION[s[2]][u]) #generate new state
			if u == 'sp' or u == 'ss':
				allowed_modes_for_motion = MODES_MOTION_ALLOWED[s[0]]
				STATE_TRANSITION_MODEL[s][u] = (s[0], s[1], s[2])
				for m in allowed_modes_for_motion:
					if m == s[2]: #make sure that the allowed mode matches the mode in the state s. If it doens't no motion will happen
						if m != 't':
							if TRANSITION_FOR_ACTION[rgc][u][m] == 'next':
								new_loc_next = LOCATIONS[min(LOCATIONS.index(s[0]) + 1, len(LOCATIONS)-1 )]
								STATE_TRANSITION_MODEL[s][u] = (new_loc_next, s[1], s[2])
							elif TRANSITION_FOR_ACTION[rgc][u][m] == 'prev':
								new_loc_prev = LOCATIONS[max(LOCATIONS.index(s[0]) - 1, 0 )]
								STATE_TRANSITION_MODEL[s][u] = (new_loc_prev, s[1], s[2])
						elif m == 't':
							new_theta = s[1]
							if TRANSITION_FOR_ACTION[rgc][u][m] == 'next' and s[1] == 0:
								new_theta = min(PI/2, s[1] + PI/2)
							elif TRANSITION_FOR_ACTION[rgc][u][m] == 'prev' and s[1] == PI/2:
								new_theta = max(0, s[1] - PI/2)

							STATE_TRANSITION_MODEL[s][u] = (s[0], new_theta, s[2])	
			
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
	robot_init_orientation = 0
	goal_orientation = PI/2
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

def create_optimal_next_state_dict():
	for s in STATES:
		if LOCATIONS.index(s[0]) < LOCATION_OF_TURN or LOCATIONS.index(s[0]) > LOCATION_OF_TURN:
			if s[2] not in MODES_MOTION_ALLOWED[s[0]]: #if not in the proper mode, switch to the mode
				if len(MODES_MOTION_ALLOWED[s[0]]) == 1:
					OPTIMAL_NEXT_STATE_DICT[s] = (s[0],s[1],MODES_MOTION_ALLOWED[s[0]][0])
			else: #if already in proper mode, them move!
				OPTIMAL_NEXT_STATE_DICT[s] = (LOCATIONS[min(LOCATIONS.index(s[0]) + 1, NUM_LOCATIONS)], s[1], s[2])
		elif LOCATIONS.index(s[0]) == LOCATION_OF_TURN: #at the location of turning, gotta deal with both turning and then moving in the allowed linear mode
			if s[2] != 't':
				if s[1] == 0: #haven't turned, is not in 't'
					OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1], 't')
				else: #have turned is not in 't'
					if s[2] == MODES_MOTION_ALLOWED[s[0]][0]:
						OPTIMAL_NEXT_STATE_DICT[s] = (LOCATIONS[min(LOCATIONS.index(s[0]) + 1, NUM_LOCATIONS)], s[1], s[2])
					else:
						OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1], MODES_MOTION_ALLOWED[s[0]][0]) 
			else:
				#already in turning mode
				if s[1] != PI/2: #check if turned. 
					OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1] + PI/2, s[2])
				else:
					if s[2] == MODES_MOTION_ALLOWED[s[0]][0]:
						OPTIMAL_NEXT_STATE_DICT[s] = (LOCATIONS[min(LOCATIONS.index(s[0]) + 1, NUM_LOCATIONS)], s[1], s[2])
					else:
						OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1], MODES_MOTION_ALLOWED[s[0]][0])
					
def generate_optimal_control_dict():
	for s in OPTIMAL_NEXT_STATE_DICT:
		sp = OPTIMAL_NEXT_STATE_DICT[s]
		OPTIMAL_ACTION_DICT[s] = [k for k,v in STATE_TRANSITION_MODEL[s].items() if v == sp]
		OPTIMAL_ACTION_DICT[s] = TRUE_COMMAND_TO_ACTION[OPTIMAL_ACTION_DICT[s][0]]

def init_p_ui_given_a():
	for k in TRUE_ACTION_TO_COMMAND.keys():
		P_UI_GIVEN_A[k] = collections.OrderedDict()
		for u in LOW_LEVEL_COMMANDS:
			if u == TRUE_ACTION_TO_COMMAND[k]:
				P_UI_GIVEN_A[k][u] = 1.0
			else:
				P_UI_GIVEN_A[k][u] = np.random.random()*UI_GIVEN_A_NOISE

		normalization_constant = sum(P_UI_GIVEN_A[k].values())
		P_UI_GIVEN_A[k] = collections.OrderedDict({u:(v/normalization_constant) for u, v in P_UI_GIVEN_A[k].items()})
	
def init_p_um_given_ui():
	for i in LOW_LEVEL_COMMANDS: #ui
		P_UM_GIVEN_UI[i] = collections.OrderedDict()
		for j in LOW_LEVEL_COMMANDS: #um
			if i == j:
				P_UM_GIVEN_UI[i][j] = 1.0
			else:
				P_UM_GIVEN_UI[i][j] = np.random.random()*UM_GIVEN_UI_NOISE

		normalization_constant = sum(P_UM_GIVEN_UI[i].values())
		P_UM_GIVEN_UI[i] = collections.OrderedDict({u:(v/normalization_constant) for u, v in P_UM_GIVEN_UI[i].items()})


#SIMULATED HUMAN
def sample_a_given_s(s): #this is essentially a look up from optimal_action_dict
	assert s in OPTIMAL_ACTION_DICT, "Error in key. Current state not in optimal action dict"
	a = OPTIMAL_ACTION_DICT[s]
	return a

def sample_ui_given_a(a): #sample from 
	p_vector = P_UI_GIVEN_A[a].values() #list
	ui_index_vector = np.random.multinomial(1, p_vector)
	ui_index = np.nonzero(ui_index_vector)[0][0]
	ui = P_UI_GIVEN_A[a].keys()[ui_index]
	return ui

def sample_um_given_ui(ui):
	p_vector = P_UM_GIVEN_UI[ui].values() #list
	um_index_vector = np.random.multinomial(1, p_vector)
	um_index = np.nonzero(um_index_vector)[0][0]
	um = P_UM_GIVEN_UI[ui].keys()[um_index]
	return um

def sample_sp_given_s_um(s, u):
	if u is not None:
		sp = STATE_TRANSITION_MODEL[s][u]
		return sp
	else:
		return s

def compute_p_ui_given_um(a, um):
	global P_UI_GIVEN_UM #need to use global because we are modifying a global dict
	for ui in LOW_LEVEL_COMMANDS:
		P_UI_GIVEN_UM[ui] = P_UM_GIVEN_UI[ui][um] * P_UI_GIVEN_A[a][ui]
	normalization_constant = sum(P_UI_GIVEN_UM.values())
	for u in P_UI_GIVEN_UM.keys():
		P_UI_GIVEN_UM[u] = P_UI_GIVEN_UM[u]/normalization_constant

def infer_intended_commands(a, um):
	compute_p_ui_given_um(a, um)
	p_ui_given_um_vector = np.array(P_UI_GIVEN_UM.values())
	p_ui_given_um_vector = p_ui_given_um_vector + np.finfo(p_ui_given_um_vector.dtype).tiny
	u_intended = P_UI_GIVEN_UM.keys()[np.argmax(p_ui_given_um_vector)]
	uniform_distribution = np.array([1.0/p_ui_given_um_vector.size]*p_ui_given_um_vector.size)
	max_entropy = -np.dot(uniform_distribution, np.log2(uniform_distribution))
	normalized_h_of_p_ui_given_um = -np.dot(p_ui_given_um_vector, np.log2(p_ui_given_um_vector))/max_entropy 
	print "UINTENDED, UM", u_intended, um
	if u_intended != um:
		#check entropy to decide whether to intervene or not
		if normalized_h_of_p_ui_given_um < ENTROPY_THRESHOLD: #intervene
			print 'INTERVENED, LOW ENTROPY'
			if ASSISTANCE_TYPE == AssistanceType.Filter:
				u_corrected = None
			elif ASSISTANCE_TYPE == AssistanceType.Corrective:
				u_corrected = u_intended
		else:
			print 'NOT INTERVENED, HIGH ENTROPY'
			u_corrected = um #Maybe keep this as None? because u intended is not same as um? 
			
	else:
		print 'u_intended same as um, no need to do anything'
		u_corrected = um
	
	return u_corrected

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
	create_optimal_next_state_dict()
	generate_optimal_control_dict()
	
	init_p_ui_given_a()
	init_p_um_given_ui()
	current_state = (LOCATIONS[0], 0, start_mode)
	num_steps = 0
	while current_state[0] != LOCATIONS[-1]:
		a = sample_a_given_s(current_state)
		ui = sample_ui_given_a(a)
		um = sample_um_given_ui(ui)
		print "S, A, UI, UM", current_state, a, ui, um
		#insert correction system here!
		if IS_ASSISTANCE:
			um = infer_intended_commands(a, um)
		
		next_state = sample_sp_given_s_um(current_state, um)
		num_steps += 1
		print "U_APPLIED, SP ", um, next_state
		print "     "
		current_state = next_state
	
	print "TOTAL NUM STEPS ", num_steps
	

if __name__ == '__main__':
	main()