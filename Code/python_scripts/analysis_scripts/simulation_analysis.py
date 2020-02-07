import pickle
import sys
sys.path.append('../utils_ms')
from utils_ms import StartDirection, AssistanceType
import os
from IPython import embed
import numpy as np
import matplotlib
import argparse

def check_accuracy_of_corrected_um(simulation_files):
	accuracy_list = []
	for i, f in enumerate(simulation_files):
		print 'PROCESSING FILE ', i
		with open(f, 'rb') as fp:
			simulation_result = pickle.load(fp)

		assert 'combination_dict' in simulation_result
		assert 'is_assistance' in simulation_result['combination_dict']
		assert 'assistance_type' in simulation_result['combination_dict']
		if simulation_result['combination_dict']['is_assistance']:
			if simulation_result['combination_dict']['assistance_type'] == AssistanceType.Corrective:
				match_list = []
				for rep_num, v in simulation_result['trials'].items():
					match_list.extend(v['assistance_match_with_ground_truth'])
			else:
				continue
		else:
			continue
		accuracy_list.append(float(sum(match_list))/len(match_list))
	print 'OVERALL ACCURACY', sum(accuracy_list)/len(accuracy_list)

def perform_analysis(args):
	simulation_results_dir = args.simulation_results_dir
	simulation_files = os.listdir(simulation_results_dir)
	simulation_files = [os.path.join(simulation_results_dir, f) for f in simulation_files]
	check_accuracy_of_corrected_um(simulation_files)


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--simulation_results_dir', dest='simulation_results_dir',default=os.path.join(os.path.dirname(os.getcwd()), 'simulation_scripts', 'simulation_results'), help="The directory where the simulation trials will be stored")
	args = parser.parse_args()
	perform_analysis(args)
