#ifndef CONVERTER_H
#define CONVERTER_H

// Loader
#include "../util/Loader.h"

// Frame
#include "Frame.h"
// KeyFrame
#include"KeyFrame.h"
// MapPoint
#include"MapPoint.h"
// Map
#include "Map.h"

// CFAR
#include "cfar.h"

// Standard library
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <math.h>
#include <sys/stat.h>
#include <cstdlib>
#include <time.h>
#include <chrono> 

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

// Opencv header
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
#include "opencv2/videostab/global_motion.hpp"

// g2o
#include <g2o/types/slam2d/types_slam2d.h> // vertex type
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>


// g2o::SE2 to cv::Mat
cv::Mat toCvMat(const g2o::SE2 se2);

// Eigen::Vector2d to cv::Mat
cv::Mat toCvMat(const Eigen::Vector2d vec);

// cvMat2Eigen
Eigen::Isometry2d cvMat2Eigen(const cv::Mat T_cv );


// Image points coordinate to local robot frame coordinate
cv::Point2d imageFrame2LocalFrame(int rows, int cols, double scale, cv::Point2d image_point);

// Local robot frame coordinate to image points coordinate 
cv::Point2d localFrame2ImageFrame(int rows, int cols, double scale, cv::Point2d local_point);

// GPS to Cartesian coordinate
std::vector<double> gps2Cartesian(double lat, double lon);

// XYZRPY to Eigen Transformation matrix
Eigen::Matrix4d xyzrpy2mat(double x, double y, double z, double roll, double pitch, double yaw);

// Global mappoint to local phi and rho
void MapPoint2LocalPhiRho(MapPoint* pMP, cv::Mat Tcw, double& phi, double& rho);
#endif