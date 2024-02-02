import os
import sys
import csv
import glob
import argparse
import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt

from util import writeLogLine, getLoopClosure

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
logfile = 'oxford-200.log'
file = open(logfile, "w")
images = []
j = 0
i = 0
radar_images_path = os.path.join(os.path.join(args.dir, 'radar/'), '*.png')
for j, img in enumerate(sorted(glob.glob(radar_images_path))):
    images.append(img)
    if j >= 1000 and j < 1201 and j % 1 == 0:
        line = writeLogLine(j, xs_gt, ys_gt, yaws_gt, img)
        # print(line)
        file.write(line)
        i+=1


###################################### Generate reference and query frames seperately in two log files
'''
Reference frames
'''
logfile_reference = 'oxford-reference.log'
file_reference = open(logfile_reference, "w")
'''
query frames
'''
logfile_query = 'oxford-query.log'
file_query = open(logfile_query, "w")

j = 0
for j, img_reference in enumerate(images):
    if j < 5000 and j % 10 == 0:
        # Find the closeset loop closure
        match = getLoopClosure(j, xs_gt, ys_gt)
        if match != -1:
            img_query = images[match]
            reference_line = writeLogLine(j, xs_gt, ys_gt, yaws_gt, img_reference)   
            print(reference_line)     
            query_line = writeLogLine(match, xs_gt, ys_gt, yaws_gt, img_query)
            print(query_line)
            file_reference.write(reference_line)        
            file_query.write(query_line)



# Plot
plt.plot([0], [0], 'ro')
plt.plot(xs_gt, ys_gt, 'b', label='Groundtruth')


plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc=3,
           ncol=2, mode="expand", borderaxespad=0.)
plt.show()