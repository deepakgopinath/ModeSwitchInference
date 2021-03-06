
import os
import pickle
import itertools
from IPython import embed
import argparse
import collections
import sys
sys.path.append('../utils_ms')
from utils_ms import StartDirection, AssistanceType, PI


START_MODE_DICT = {StartDirection.X: {-1: 't', 1: 'y'},
              StartDirection.Y: {-1: 'x', 1: 't'}}


#experiment params
RG_CONFIGS = ['tr', 'tl', 'bl', 'br']
NUM_TURNS = [1, 2, 3]
START_DIRECTION = [StartDirection.X, StartDirection.Y]
ASSISTANCE_TYPE = [AssistanceType.Filter, AssistanceType.Corrective, AssistanceType.No_Assistance]
START_MODE = [-1, 1]
UI_GIVEN_A_NOISE = [i/10.0 for i in range(1, 9, 2)]
UM_GIVEN_UI_NOISE = [i/10.0 for i in range(1, 9, 2)]
ENTROPY_THRESHOLD = [i/10.0 for i in range(5, 10)]
TARGET_ORIENTATIONS = [-PI/2, PI/2]

def generate_simulation_trials(args):
	simulation_trial_dir = args.simulation_trial_dir
	if not os.path.exists(simulation_trial_dir):
		os.makedirs(simulation_trial_dir)

	trial_combinations = []
	i = 0
	for (r_to_g_config, num_turns, start_direction, assistance_type, start_mode, ui_given_a_noise, um_given_ui_noise, entropy_threshold, target_orientation) in itertools.product(RG_CONFIGS, NUM_TURNS, START_DIRECTION, ASSISTANCE_TYPE, START_MODE, UI_GIVEN_A_NOISE, UM_GIVEN_UI_NOISE, ENTROPY_THRESHOLD, TARGET_ORIENTATIONS):
		combination_dict = collections.OrderedDict()
		combination_dict['r_to_g_config'] = r_to_g_config
		combination_dict['num_turns'] = num_turns
		combination_dict['start_direction'] = start_direction
		combination_dict['assistance_type'] = assistance_type
		combination_dict['start_mode'] = START_MODE_DICT[start_direction][start_mode]
		combination_dict['ui_given_a_noise'] = ui_given_a_noise
		combination_dict['um_given_ui_noise'] = um_given_ui_noise
		combination_dict['entropy_threshold'] = entropy_threshold
		combination_dict['target_orientation'] = target_orientation

		print "Creating trial number ", i
		with open(os.path.join(simulation_trial_dir, str(i) + '.pkl'), 'wb') as fp:
			pickle.dump(combination_dict, fp)

		i += 1

	embed()



if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--simulation_trial_dir', dest='simulation_trial_dir',default=os.path.join(os.getcwd(), 'simulation_trial_dir'), help="The directory where trials will be stored are")
	args = parser.parse_args()
	generate_simulation_trials(args)
