import csv
import matplotlib.pyplot as plt
import os
import argparse

parser = argparse.ArgumentParser(description='param')
parser.add_argument('--sequence', type=int, help='number')
args = parser.parse_args()

sequence = args.sequence

if sequence == 1:
	pose_file = '/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/2019-01-10-11-46-21-radar-oxford-10k/ro/keyframe/23/keyframe_pose.csv' # 001
elif sequence == 2:
	pose_file = '/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/2019-01-10-12-32-52-radar-oxford-10k/ro/keyframe/23/keyframe_pose.csv' # 002
elif sequence == 3:
	pose_file = '/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/2019-01-11-14-02-26-radar-oxford-10k/ro/keyframe/23/keyframe_pose.csv' # 003
elif sequence == 4:
	pose_file = '/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/2019-01-16-11-53-11-radar-oxford-10k/ro/keyframe/23/keyframe_pose.csv' # 004 
elif sequence == 5:
	pose_file = '/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/2019-01-17-13-26-39-radar-oxford-10k/ro/keyframe/23/keyframe_pose.csv' # 005
elif sequence == 6:
	pose_file = '/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/2019-01-18-14-14-42-radar-oxford-10k/ro/keyframe/23/keyframe_pose.csv' # 006
elif sequence == 7:
	pose_file = '/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/2019-01-18-14-46-59-radar-oxford-10k/ro/keyframe/23/keyframe_pose.csv' # 007
elif sequence == 8:
	pose_file = '/media/hong/DiskC/Oxford_Radar_RobotCar_Dataset/2019-01-18-15-20-12-radar-oxford-10k/ro/keyframe/23/keyframe_pose.csv' # 008

elif sequence == 9:
	pose_file = '/media/hong/DiskB/MulRan_Dataset/DCC01/ro/keyframe/23/keyframe_pose.csv' # DCC01
elif sequence == 10:
	pose_file = '/media/hong/DiskB/MulRan_Dataset/DCC02/ro/keyframe/23/keyframe_pose.csv' # DCC02
elif sequence == 11:
	pose_file = '/media/hong/DiskB/MulRan_Dataset/DCC03/ro/keyframe/23/keyframe_pose.csv' # DCC02
elif sequence == 12:
	pose_file = '/media/hong/DiskB/MulRan_Dataset/KAIST01/ro/keyframe/23/keyframe_pose.csv' # KAIST01
elif sequence == 13:
	pose_file = '/media/hong/DiskB/MulRan_Dataset/KAIST02/ro/keyframe/23/keyframe_pose.csv' # KAIST02
elif sequence == 14:
	pose_file = '/media/hong/DiskB/MulRan_Dataset/KAIST03/ro/keyframe/23/keyframe_pose.csv' # KAIST03
elif sequence == 15:
	pose_file = '/media/hong/DiskB/MulRan_Dataset/Riverside01/ro/keyframe/23/keyframe_pose.csv' # Riverside01
elif sequence == 16:
	pose_file = '/media/hong/DiskB/MulRan_Dataset/Riverside02/ro/keyframe/23/keyframe_pose.csv' # Riverside02
elif sequence == 17:
	pose_file = '/media/hong/DiskB/MulRan_Dataset/Riverside03/ro/keyframe/23/keyframe_pose.csv' # Riverside03


abs_dir = os.path.dirname(os.path.abspath(pose_file))
image_save_name = abs_dir + '/traj.png'
# opening the CSV file
x = []
y = []
with open(pose_file, mode ='r')as file:
	# reading the CSV file
	csvFile = csv.reader(file)
	first_line = True
	# displaying the contents of the CSV file
	for line in csvFile:
		# print(line)
		if first_line:
			first_line = False
		else:
			x.append(line[1])
			y.append(line[2])

plt.plot(x,y)
plt.show()



