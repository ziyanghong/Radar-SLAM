import os
import sys
import csv
import glob
import argparse
import numpy as np
from math import cos, sin, pi
from sklearn.neighbors import KDTree


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
def writeLogLine(j, xs_gt, ys_gt, yaws_gt, img):
	line ='RADAR '          \
	+'0 '                   \
	+'1.5708 '              \
	+ '6.28318 '            \
	+ '0.0157 '             \
	+'87.5 '                \
	+ '0.0432 '             \
	+ '0 '                  \
	+ '0 '                  \
	+ '0 '                  \
	+ str(j) + ' '             \
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
	return line

def getLoopClosure(i, xs_gt, ys_gt):
	j = -1
	xs = np.asarray(xs_gt).reshape((len(xs_gt),1))
	ys = np.asarray(ys_gt).reshape((len(ys_gt),1))
	poses = np.hstack((xs, ys))		
	tree = KDTree(poses, leaf_size=2)              # doctest: +SKIP
	indexes, dists = tree.query_radius([poses[i,:]], r=5, return_distance=True, sort_results=True)
	inds = indexes[0]
	if inds.shape[:-1] > 0:
		for ind in inds:
			if abs(ind - i) > 400:
				print("The reference frame: ",i, " has loop closure in frame: ", ind)
				j = ind
				break

	match = j
	return match

# rng = np.random.RandomState(0)
# X = rng.random_sample((10, 3))  # 10 points in 3 dimensions
# print(X.ndim)
# print(X)
# print(X.shape)
# tree = KDTree(X, leaf_size=2)     # doctest: +SKIP	
# ind = tree.query_radius([X[0,:]], r=0.3)  # doctest: +SKIP
# print(ind)