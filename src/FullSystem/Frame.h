#ifndef FRAME_H
#define FRAME_H

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
#include <thread>


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
#include "opencv2/xfeatures2d/nonfree.hpp"

#include "MapPoint.h"
#include "../util/Loader.h"
#include "KDTree.h"
#include "Algorithms.h"
#include "../util/polarToCartesian.h"
#include "Converter.h"
#include "FeatureTracker.h"
#include "Feature.h"
// namespace ROAM{

class KDTree;
class KDNode;
class MapPoint;
class KeyFrame;
class Feature;

class Frame{

public:
    //-----------------------------------------------Functions-----------------------------------------------
    Frame();
    Frame(std::size_t frame_index, double timestampCurrent, double timestampPrevious, 
            const cv::Mat &PolarImage, const cv::Mat &CartImage, ConfigLoader config_loader);
    ~Frame();

    // M2DP 
    void ComputeGlobalDescriptor();

    // Get Functions
    cv::Mat GetPose();
    cv::Mat GetFeature();
    cv::Mat GetPointcloud();

    cv::Mat GetPoseInverse();
    cv::Mat GetRT();
    KeyFrame* GetReferenceKeyFrame();
    Eigen::Matrix<double, 3, 1> GetVelocity();

    // Set Functions
    void SetReferenceKeyFrame(KeyFrame* pReferenceKeyFrame);
    void SetKeypoints(const std::vector<cv::KeyPoint> &keypoints);
    void SetDescriptors(const cv::Mat &descriptors);
    void SetRT(const cv::Mat &RT_);
    void SetPose(const cv::Mat &Tcw_);
    void SetVelocity(const Eigen::Matrix<double, 3, 1> v);
    void ExtractKeyPoints(const std::size_t& FrameId, const cv::Mat &PolarImage, const cv::Mat CartImage, 
                        const ConfigLoader config_loader, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);
    
    bool IsVisibleInRadius(MapPoint* pMP, double radius, double noise);
    bool IsReady();
    bool IsM2dpReady();
    //-----------------------------------------------Variables-----------------------------------------------
    int rows;
    int cols;
    double scale;

    long unsigned int mnId;

    // Number of KeyPoints.
    int N;
    cv::Mat mCartImage;
    cv::Mat mPolarImage;
    double mTimestamp;   
    double mPreviousTimestamp;
    cv::Mat mKeyPointsImage;
    cv::Mat mvDescriptors;
    std::vector<cv::KeyPoint> mvKeypoints; // This is used for tracking keyframe and local mapping optimization
    pointVec mLocalPoints; // For KD-Tree
    int nMatches;
    cv::Mat mImageMatches;
    bool mbReady;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // KD tree
    KDTree mTree;

    // Tcw estimated by SVD
    cv::Mat TcwSVD;

    // Feature Tracker variables
    std::vector<cv::Point2f> mvFeatureCoordinates;
    std::vector<Feature*> mvFeatures;
    // Velocity
    Eigen::Matrix<double, 3, 1> mVelocity;

    // number of matches 
    int mnMatchesWithLastKeyFrame;

    cv::Mat  mvPointCloud; // M2DP feature based pointcloud


    // Config loader
    ConfigLoader config_loader;    
private:
    // Pose
    cv::Mat Tcw;
    cv::Mat RT2KeyFrame;
    // Inverse Pose
    cv::Mat Twc;
    // Pointer to reference keyframe
    KeyFrame* mpReferenceKeyFrame;
    cv::Mat mFeature; // M2DP feature
    bool mbM2dpReady; // M2DP ready flag


};

// } // namespace ROAM

#endif