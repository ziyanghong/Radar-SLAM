import cv2 as cv
import os
import sys
import glob
import numpy as np
from matplotlib import pyplot as plt
import time
from random import shuffle
from ssc import *

def extractFeature(image,mask,feature):
	t0= int(round(time.time() * 1000))

	# Subtract mean
	subMeanImg =  np.ones((image.shape), np.uint8)*int(np.mean(image))
	print subMeanImg.shape
	image  = cv.subtract(image,subMeanImg)

	kpts, des = feature.detectAndCompute(image, mask)

	# keypoints should be sorted by strength in descending order before feeding to SSC to work correctly
	# shuffle(kpts)  # simulating sorting by score with random shuffle
	kpts = sorted(kpts, key=lambda x: x.response, reverse=True)
	

	selected_keypoints,result_list = ssc(kpts, 100, 0.1, image.shape[1], image.shape[0])
	selected_descriptors = des[result_list]
	t1 = int(round(time.time() * 1000))
	# img3 = cv.drawKeypoints(image, selected_keypoints, np.array([]))
	# cv.imshow('Selected keypoints', img3)
	# cv.waitKey(0)	
	print("Time elapsed: ", t1 - t0) 

	return selected_keypoints, selected_descriptors

def updateMask(prevImg, currImg, mask):

	return mask

# Path
folder = '/home/hong/Documents/Oxford_Radar_RobotCar_Dataset/'
sequence= '2019-01-17-13-26-39-radar-oxford-10k/701_radar_cart'
saveMatchesPath = '/home/hong/Desktop/matches/'
saveKptPath = '/home/hong/Desktop/keypoints/'
# All images
imageList = []
means = []
for i in sorted(glob.glob(folder + sequence+ '/*.png')):
	imageList.append(i)

# # Visualize intensity mean
# for i in range(200):
# 	img = cv.imread(imageList[i])
# 	means.append(cv.mean(img))
# print 'Finished computing mean'
# plt.plot(means)
# plt.ylabel('mean of intensity')
# plt.show()	


# for i in range(len(imageList)):
k = 1
offset = 5000
mask = np.ones((701, 701),np.uint8)
# Initiate detector and descriptor
# descriptor = cv.xfeatures2d.SIFT_create()
descriptor = cv.xfeatures2d.SURF_create(700)
# descriptor = cv.ORB_create()

counter = 1

# FLANN parameters
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)   # or pass empty dictionary
flann = cv.FlannBasedMatcher(index_params,search_params)


for i in range(k,k+offset):
	print i
	if counter < 2:
		prevImg = cv.imread(imageList[i],0)
		prevKpt, prevDes = extractFeature(prevImg, mask, descriptor)
		counter += 1
	elif counter >= 2:
		currImg = cv.imread(imageList[i],0)
		currKpt, currDes = extractFeature(currImg, mask, descriptor)

		t0= int(round(time.time() * 1000))
		matches = flann.knnMatch(prevDes,currDes,k=2)
		counter+=1

		# ratio test as per Lowe's paper
		good = []
		for m,n in matches:
			if m.distance < 0.7*n.distance:
				good.append(m)
		print("KNN matching time elapsed: ", int(round(time.time() * 1000)) - t0) 

     	# Refine with homography
		src_pts = np.float32([ prevKpt[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
		dst_pts = np.float32([ currKpt[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
		M, maskHomo = cv.findHomography(src_pts, dst_pts, cv.RANSAC,5.0)
		t1 = int(round(time.time() * 1000))
		print("homography matching time elapsed: ", t1 - t0) 

		# Draw matches
		matchesMask = maskHomo.ravel().tolist()		
		draw_params = dict(matchColor = (0,255,0),
						singlePointColor = None,
						matchesMask = matchesMask,
						flags = 2)   
		imgMatches = cv.drawMatches(prevImg,prevKpt,currImg,currKpt,good,None,**draw_params)	
		cv.imwrite(saveMatchesPath + str(counter).zfill(6) + '.jpg', imgMatches)

		prevImg = currImg
		prevKpt = currKpt
		prevDes = currDes
		# prevImageKpt =cv.drawKeypoints(prevImg, prevKpt,np.array([]))
		currImageKpt =cv.drawKeypoints(currImg, currKpt,np.array([]))
		cv.imwrite(saveKptPath + str(counter).zfill(6) + '.jpg', currImageKpt)
		# plt.subplot(221)
		# plt.imshow(prevImageKpt)
		# plt.subplot(222) 
		# plt.imshow(currImageKpt)
		# plt.subplot(212)
		# plt.imshow(imgMatches)
		# plt.show()

	else:
		mask    = updateMask(prevImg, currImg, mask)
