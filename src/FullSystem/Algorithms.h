#ifndef ALGORITHMS_H
#define ALGORITHMS_H

// Standard library
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <math.h>
#include <sys/stat.h>
#include <cstdlib>
#include <time.h>
#include <chrono>
#include <assert.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Sparse>

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

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

// Converter
#include"Converter.h"

// Loader
#include "../util/Loader.h"

// icp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/features/fpfh.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/keypoints/iss_3d.h>

// Frame
#include "Frame.h"

// Update the covariance of the robot pose
cv::Mat updateCovariance(cv::Mat Pk_prev, double *Xk, double *Uk, double *processNoises);

// Sorting algorithms for ordering the keyframe index
void sortrows(std::vector<std::vector<double>>& matrix, int col);

// Estimate motion using SVD based on image points
cv::Mat estimateMotionSVD(const ConfigLoader &config_loader, std::vector<cv::Point2f> prev, std::vector<cv::Point2f> curr, bool &trackOK);

// Estimate motion using ICP based on image points
cv::Mat estimateMotionICP(const ConfigLoader config_loader, std::vector<cv::Point2f> prev, std::vector<cv::Point2f> curr, bool &trackOK);
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event,
                       void* nothing);

// Estimate motion using FPFH features
cv::Mat estimatePointCloudMotionFPFH(const ConfigLoader config_loader,
                                    cv::Mat &prev,
                                    cv::Mat &curr, bool &trackOK);

// Estiate motion using ICP based on local point clouds
cv::Mat estimatePointCloudMotionICP(const ConfigLoader config_loader, cv::Mat &prev, cv::Mat &curr, bool &trackOK);


// Find maximum inliers
std::vector<cv::DMatch> findMaximumInliers(const float dis_thres, const std::vector<cv::DMatch> unary_matches,
                                           std::vector<cv::Point2f> queryPoints, std::vector<cv::Point2f> trainPoints);

/* 
Find the maximum clique
More information at: http://www.sicmm.org/~konc
*/
int *findMaximumClique(std::set<int> v, std::multimap<int, int> e, int &_qsize);

// Detect keypoints
void detectKeypoint(const ConfigLoader config_loader, cv::Mat cart_image, std::vector<cv::KeyPoint> &keypoints);

// Extract SURF descriptors
void extractSurfDescriptor(const ConfigLoader config_loader, cv::Mat image, std::vector<cv::KeyPoint> keypoints, cv::Mat &descriptors);

// Extract SURF feature with ANMS
void FeatureExtractionANMS(cv::Mat cart_image, ConfigLoader config_loader, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);


// keypoints matching
void ImageKeypointMatching(const ConfigLoader config_loader, cv::Mat cart_image, 
                    std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2,
                    cv::Mat descriptors1, cv::Mat descriptors2, 
                    std::vector<cv::Point2f> &curr, std::vector<cv::Point2f> &prev, std::vector<cv::DMatch> &_matches);

// dynamic targets removal rejections in tracking
void DynamicTargetRemoval(const ConfigLoader config_loader, std::vector<cv::DMatch> &matches, 
                            std::vector<cv::Point2f> prevPoints, std::vector<cv::Point2f> currpoints);

// struct structSFM
// {

//     using landmark_idx_t = size_t;
//     using img_idx_t = size_t;
//     using kp_encoded_idx = size_t;

//     std::map<kp_encoded_idx, int> keypoint_landmark_pairs; 
//     std::vector<std::vector<kp_encoded_idx>> landmarks;
//     std::vector<std::vector<kp_encoded_idx>> landmarks_refined;
//     std::vector<cv::Point2d> landmarks_Tcw;

//     bool kpt_encoded_idx_exist(kp_encoded_idx kp_encoded_idx) { return keypoint_landmark_pairs.count(kp_encoded_idx) > 0; }

// };

// // trackPoints
// void trackPoints(const ConfigLoader config_loader, std::vector<Frame*> localFrames, structSFM &struct_sfm);
// void encodeKeypointIndex( std::vector<Frame*> localFrames, const size_t frame_idx, const size_t kpt_idx, size_t &IndexEncode);
// void decodeKeypointIndex( std::vector<Frame*> localFrames, const size_t kpt_idx_encoded, int &IndexDecode, int &FrameIndexDecode);


#endif