import os
import sys
import csv
import glob
import argparse
import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description='Play back radar data from a given directory')
parser.add_argument('-dir', type=str, help='Directory containing radar data.')

args = parser.parse_args()




########################################### groundtruth
Tcw = np.eye(3, dtype=float)
xs_gt = [0.0]
ys_gt = [0.0]
yaws_gt = [0.0]
yaw_global =  0.0
groundtruth_path = os.path.join(os.path.join(args.dir, 'gt/radar_odometry.csv'))
with open(groundtruth_path) as csvfile:
    readCSV = csv.reader(csvfile, delimiter=',')
    i = 0
    for row in readCSV:
    	if i < 1:
    		print("skip first line of gt")
    	else:
            yaw = float(row[7])
            yaw_global = yaw_global + yaw
            x = float(row[2]) # 
            y = float(row[3]) # 
            RT = np.asarray([[cos(yaw), -sin(yaw), x],
    						 [sin(yaw),  cos(yaw), y],
    						 [0       ,         0, 1]])
            Tcw = Tcw.dot(RT)
            line = str(Tcw[0,2]) + ' ' + str(Tcw[1,2]) + '\n'
            xs_gt.append(Tcw[0,2])
            ys_gt.append(Tcw[1,2])
            yaws_gt.append(yaw_global)

        i+=1




###################################### Generate log file in CARMEN format
logfile = 'oxford.log'
file = open(logfile, "w")
j = 0
i = 0
radar_images_path = os.path.join(os.path.join(args.dir, 'radar/'), '*.png')
for img in sorted(glob.glob(radar_images_path)):
    if j < 7500 and j % 1 == 0:
        line ='ROBOTLASER1 '    \
        +'0 '                   \
        +'1.5708 '              \
        + '6.28318 '            \
        + '0.0157 '             \
        +'87.5 '                \
        + '0.0432 '             \
        + '0 '                  \
        + '0 '                  \
        + '0 '                  \
        + img + ' '             \
        +'701 '                 \
        +'701 '                 \
        + '87.5 '               \
        + str(xs_gt[j]) + ' '   \
        + str(ys_gt[j]) + ' '   \
        + str(yaws_gt[j]) + ' ' \
        + str(xs_gt[j]) + ' '   \
        + str(ys_gt[j]) + ' '   \
        + str(yaws_gt[j]) + ' ' \
        + '0.0 '                \
        + '0.0 '                \
        + '0.0 '                \
        + '0.0 '                \
        + '0.0 '                \
        + '0.0 '                \
        + 'x\n'    
        # print(line)
        file.write(line)
        i+=1

    j+=1
print('Total number of images%i', i)
'''
    'RADAR '                # sensorName
    +'0 '                   # laserType
    +'0 '                   # start
    + '6.28318 '            # fov
    + '0 '                  # resolution
    +'87.5 '                # maxRange
    + '0.0 '                # accuracy
    + '0 '                  # remissionMode
    + '0 '                  # number
    + '0 '                  # remissionNumber   
    + img + ' '             # radarImagePath
    +'701 '                 # rows
    +'701 '                 # cols
    + '87.5 '               # scale
    + str(xs[i]) + ' '      # laserPose.x
    + str(ys[i]) + ' '      # laserPose.y
    + str(yaws_gt[i]) + ' ' # laserPose.theta
    + str(xs[i]) + ' '      # robotPose.x
    + str(ys[i]) + ' '      # robotPose.y
    + str(yaws_gt[i]) + ' ' # robotPose.theta
    + '0.0 '                # tv
    + '0.0 '                # rv
    + '0.0 '                # forward_safety_dist
    + '0.0 '                # side_safety_dist
    + '0.0 '                # turn_axis
    + '0.0 '                # timestamp
    + 'x'                   # robotName
'''

# Plot
plt.plot([0], [0], 'ro')
plt.plot(xs_gt, ys_gt, 'b', label='Groundtruth')


plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
plt.show()