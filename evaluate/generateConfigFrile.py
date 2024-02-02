import numpy as np
import os
import sys
from ruamel import yaml
import glob
import shutil
import cv2



def createROfolder(dataset_path, sequence, folder_option):
	idx = 1
	ro_folder = dataset_path + sequence + '/ro_' + str(idx)
	# check is the directory exsits
	while(os.path.isdir(ro_folder)):
		idx = idx + 1
		ro_folder = dataset_path + sequence + '/ro_' + str(idx)

	old_folder = dataset_path + sequence + '/ro'
	new_folder = dataset_path + sequence + '/ro_' + str(idx)
	if 'delete' in folder_option:
		print 'Delete old folder:'
		print old_folder
		shutil.rmtree(old_folder)
	elif 'rename' in folder_option:
		print 'Rename ro directory:'
		print new_folder
		os.rename(old_folder, new_folder) 		
	else:
		print 'n/a'
	os.mkdir(dataset_path + sequence + '/ro')	
	os.mkdir(dataset_path + sequence + '/ro/keyframe')
	os.mkdir(dataset_path + sequence + '/ro/odometry')


# -------------------------------------------------------------------------------------------------------------------------

# bash script
bash_file_name = '/home/hong/catkin_ws/src/radar_localization/evaluate/run_experiments_all_with_everything.sh'
print bash_file_name
bash_file = open(bash_file_name,'w')
bash_file.write('#!/bin/bash\n')
bash_file.write('cd /home/hong/catkin_ws/devel/lib/radar_localization\n')

# Example config files
folder = '/home/hong/catkin_ws/src/radar_localization/Example/'
new_config_folder = folder + '../testConfig/'
if os.path.exists(new_config_folder):
	shutil.rmtree(new_config_folder)
os.mkdir(new_config_folder)

n = len(sorted(glob.glob(folder+ '*.yaml')))
# print(n)
class ConfigGenerator:
	def __init__(self):
		self.nodeID = 1

	def writeConfigFile(self, path, new_path):
		i = 1
		total=0

		for file in sorted(glob.glob(folder+ '*.yaml')):
			with open(file) as f:
				content = f.readlines()
				for loopClosure in [0]:
					for treeRadius in [50]:
						for LowesRatio in [0.8]:
							for minHessian in [700]:
								for cliqueDistThres in [2]:
									new_config_name = new_config_folder + str(self.nodeID).zfill(3) + '_config.yaml'
									new_config_file = open(new_config_name, 'w')
									for line in content:
										if line.startswith('cliqueDistThres'):
											new_config_file.write('cliqueDistThres: ' + str(int(cliqueDistThres)) + '\n')
										elif line.startswith('LowesRatio'):
											new_config_file.write('LowesRatio: ' + str(float(LowesRatio)) + '\n')
										elif line.startswith('minHessian'):
											new_config_file.write('minHessian: ' + str(int(minHessian)) + '\n')
										elif line.startswith('nodeID'):
											new_config_file.write('nodeID: ' + str(int(self.nodeID)) + '\n')
										elif line.startswith('treeRadius'):
											new_config_file.write('treeRadius: ' + str(int(treeRadius)) + '\n')
										elif line.startswith('loopClosure'):
											new_config_file.write('loopClosure: ' + str(int(loopClosure)) + '\n')
										elif path in line:
											new_config_file.write(line.replace(path, new_path) + '\n')
										else:
											new_config_file.write(line)
									new_config_file.close()
									bash_file.write('./radar_localization ' + '"' + new_config_name + '"' + ' > /dev/null  &\n')
									if self.nodeID % 30 == 0:
										bash_file.write('wait && echo "Finished ' + str(self.nodeID) + ' job."' + '\n')
										i = 1
									self.nodeID = self.nodeID+1
									i = i + 1
									total = total + 1 

	# bash_file.write('wait && echo "Finished all jobs."')

# -------------------------------------------------------------------------------------------------------------------------
# dataset_path  = '/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/'
dataset_path  = '/media/hong/DiskB/MulRan_Dataset/'


folders = [dI for dI in os.listdir(dataset_path) if os.path.isdir(os.path.join(dataset_path,dI))]
# folders = ['2019-01-16-13-09-37-radar-oxford-10k']

# print(folders)
old_path = '/media/hong/DiskB/MulRan_Dataset/KAIST02/'
config = ConfigGenerator()
for new_path in folders:
	# createROfolder(dataset_path, new_path, 'delete')
	# createROfolder(dataset_path, new_path, 'keep')
	new_path = dataset_path + new_path + '/'
	print new_path

	for j in range(1):
		config.writeConfigFile(old_path, new_path)