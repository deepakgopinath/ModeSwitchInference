# import pickle
# import sys
# sys.path.append('../utils_ms')
# from utils_ms import StartDirection, AssistanceType
# import os
# from IPython import embed
# import numpy as np
# import matplotlib
# import argparse

# SIMULATION_FILES = None


# def check_accuracy_of_corrected_um(simulation_files):
# 	accuracy_list = []
# 	for i, f in enumerate(simulation_files):
# 		print 'PROCESSING FILE ', i
# 		with open(f, 'rb') as fp:
# 			simulation_result = pickle.load(fp)

# 		assert 'combination_dict' in simulation_result
# 		assert 'is_assistance' in simulation_result['combination_dict']
# 		assert 'assistance_type' in simulation_result['combination_dict']
# 		if simulation_result['combination_dict']['is_assistance']:
# 			if simulation_result['combination_dict']['assistance_type'] == AssistanceType.Corrective:
# 				match_list = []
# 				for rep_num, v in simulation_result['trials'].items():
# 					match_list.extend(v['assistance_match_with_ground_truth'])
# 			else:
# 				continue
# 		else:
# 			continue
# 		accuracy_list.append(float(sum(match_list))/len(match_list))
# 	print 'OVERALL ACCURACY', sum(accuracy_list)/len(accuracy_list)

# def data_parser():
# 	pass


# def perform_analysis(args):
# 	global SIMULATION_FILES
# 	simulation_results_dir = args.simulation_results_dir
# 	SIMULATION_FILES = os.listdir(simulation_results_dir)
# 	SIMULATION_FILES = [os.path.join(simulation_results_dir, f) for f in SIMULATION_FILES]
# 	check_accuracy_of_corrected_um(simulation_files)


# if __name__ == '__main__':
# 	parser = argparse.ArgumentParser()
# 	parser.add_argument('--simulation_results_dir', dest='simulation_results_dir',default=os.path.join(os.path.dirname(os.getcwd()), 'simulation_scripts', 'simulation_results'), help="The directory where the simulation trials will be stored")
# 	args = parser.parse_args()
# 	perform_analysis(args)

import pickle
import sys
sys.path.append('../utils_ms')
import os
import numpy as np
import matplotlib
import argparse
from utils_ms import StartDirection, AssistanceType

class DataParser(object):
    """docstring forDataParser."""
    def __init__(self, simulation_results_dir):
        super(DataParser, self).__init__()
        self.simulation_results_dir = simulation_results_dir
        self.simulation_files = os.listdir(simulation_results_dir)
        self.simulation_files = [os.path.join(self.simulation_results_dir, f) for f in self.simulation_files]

    def parse_sim_files_for_specific_criteria(self, criteria, output_keys):
        '''
        criteria is a dict with same keys as combination_dict
        output_keys should be a subset of the keys in trial['data']
        '''
        for f in self.simulation_files:
            with open(f, 'rb') as fp:
                simulation_result = pickle.load(fp)

            combination_dict = simulation_result['combination_dict']
            embed()

class SimulationAnalysis(object):
    """docstring forSimulationAnalysis."""
    def __init__(self, args):
        super(SimulationAnalysis, self).__init__()
        self.simulation_results_dir = args.simulation_results_dir
        self.data_parser = DataParser(self.simulation_results_dir)
        self.combination_dict_keys = ['r_to_g_config', 'num_turns', 'start_direction', 'assistance_type', 'start_mode', 'ui_given_a_noise', 'um_given_ui_noise', 'entropy_threshold']
        #experiment params
        self.RG_CONFIGS = ['tr', 'tl', 'bl', 'br']
        self.NUM_TURNS = [1, 2, 3]
        self.START_DIRECTION = [StartDirection.X, StartDirection.Y]
        self.ASSISTANCE_TYPE = [AssistanceType.Filter, AssistanceType.Corrective, AssistanceType.No_Assistance]
        self.START_MODE = [-1, 1]
        self.UI_GIVEN_A_NOISE = [i/10.0 for i in range(1, 9, 2)]
        self.UM_GIVEN_UI_NOISE = [i/10.0 for i in range(1, 9, 2)]
        self.ENTROPY_THRESHOLD = [i/10.0 for i in range(5, 10)]


    def perform_analysis(self):
        pass

    def _check_accuracy_of_corrected_um(self):
        criteria = collections.OrderedDict()
        for key in self.combination_dict_keys:
            criteria[key] = 'all'
        criteria['assistance_type'] = [AssistanceType.Corrective]
        output_keys = ['assistance_match_with_ground_truth']
        data = self.data_parser.parse_sim_files_for_specific_criteria(criteria, output_keys)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--simulation_results_dir', dest='simulation_results_dir',default=os.path.join(os.path.dirname(os.getcwd()), 'simulation_scripts', 'simulation_results'), help="The directory where the simulation trials will be stored")
    args = parser.parse_args()
    sa = SimulationAnalysis(args)
