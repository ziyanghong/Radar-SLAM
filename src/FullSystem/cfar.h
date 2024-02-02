#ifndef CFAR_H
#define CFAR_H

#include<vector>
#include<iostream>
#include <math.h>       
#include <chrono> 

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
// opencv
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/persistence.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/xfeatures2d.hpp"

// KD tree
#include "KDTree.h"
// fft
#include <complex>
#include <iostream>
#include <valarray>
 
const double PI = 3.141592653589793238460;
 
typedef std::complex<double> Complex;
typedef std::valarray<Complex> CArray;
 
void fft(CArray& x);

void detectPeakCFAR(const cv::Mat azimuth_scan, int azimuth_index, std::vector<cv::KeyPoint> &keypoints, int visPointSize
,const int num_train, const int num_guard, const double falseAlarmRate);

void computeDescriptor(cv::Mat cart_image, int radius, int wedgeResolution, std::vector<cv::KeyPoint> keypoints, cv::Mat &descriptors);

float area(int x1, int y1, int x2, int y2, int x3, int y3);

bool isInside(int x1, int y1, int x2, int y2, int x3, int y3, int x, int y);

#endif