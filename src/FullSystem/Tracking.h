#ifndef TRACKING_H
#define TRACKING_H

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

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

// System
#include "System.h"
// Loader
#include "../util/Loader.h"
// Mercator
#include "../util/mercator.h"
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
// Polar to Cartesian
#include "../util/polarToCartesian.h"
// Algorithms
#include"Algorithms.h"
// Matcher
#include "Matcher.h"
// Converter
#include "Converter.h"
// Viewer
#include "Viewer.h"
// KDTree
#include "KDTree.h"
// Loop Closer
#include "LoopClosing.h"
// Feature Tracker
#include "FeatureTracker.h"

// namespace ROAM
// {

class LocalMapping;
class LoopClosing;
class KeyFrame;
class Frame;
class System;
class Matcher;
class Viewer;
class KDTree;
class KDNode;

class Tracking
{
    // ---------------------------------------------------Variables--------------------------------------------------
protected:
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState = NOT_INITIALIZED;

    // System
    System* mpSystem;
    
    // Map
    Map* mpMap;  

    // Local mapper
    LocalMapping* mpLocalMapper;

    // Loop closer
    LoopClosing* mpLoopCloser;

    // Viewer
    Viewer* mpViewer;

    // Feature tracker
    FeatureTracker* mpFeatureTracker;

    // Local map
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;

    //Current matches in frame
    int mnMatchesInliers;
    int mnToMatchLocalMap;

    // Current key frame
    KeyFrame* mpCurrentKeyFrame;

    // Pose variables
    std::vector<cv::Mat> mvHistoryPoses;
    cv::Mat Tcw_init = cv::Mat::eye(3,3,CV_32F); // 3 DOF robot pose
    cv::Mat Tcw_prev = cv::Mat::eye(3,3,CV_32F); // 3 DOF robot pose
    cv::Mat Tcw_pure = cv::Mat::eye(3,3,CV_32F); // Pure SVD odometry
    cv::Mat mTcwCurrent = cv::Mat::eye(3,3,CV_32F); // 3 DOF robot pose
    cv::Mat RT_prev  = cv::Mat::eye(3,3,CV_32F); // 3 DOF relative robot pose
     
    void Initialization();
    void CreateNewKeyFrame();
    void PredictCovariance();

public:

    // Current Frame
    long unsigned int mnId;
    Frame mCurrentFrame;
    std::size_t mFrameId;
    cv::Mat mCartImage;
    cv::Mat mPolarImage;
    cv::Mat mvDescriptors;
    std::vector<cv::KeyPoint> mvKeypoints;

    // Previous Frame
    bool mbPureOdomInitilized = false;
    Frame mPreviousFrame;

    // Motion model variables
    double mCurrentTimestamp;
    double mPreviousTimestamp;
    double mVelocityX;
    double mVelocityY;
    double mVelocityYaw;

    // Robot Pose Covariance Pk and noise matrix Qk
    cv::Mat Pk;
    cv::Mat Qk;
    double processNoises[3];

    // Configuration
    ConfigLoader mConfigLoader;

    // Local Window Feature Tracker
    std::vector<Frame> mvWindowFrames;
    std::vector<cv::Mat> mvRT;
    Frame mLastFrame;

    // Localiztion
    bool mbLocalised;

public:
    Tracking(System* pSystem, Map* pMap, const ConfigLoader config_loader);
    ~Tracking();
    void GrabSensorInput(const std::size_t &frame_id, const double timestamp, 
                        const cv::Mat &polar_image, const cv::Mat &cart_image);
                        
    int GetTrackingState();
    std::vector<Frame *> GetLocalWindowFrames();


    // ------------------------------------------Set and Get Functions------------------------------------------------
    void SetLocalMapper(LocalMapping* pLocalMapper);   
    void SetLoopCloser(LoopClosing* pLoopCloser);
    void SetViewer(Viewer* pViewer);
    void SetFeatureTracker(FeatureTracker *pFeatureTracker);
    void SetLocalised();

    // ------------------------------------------Pure Odometry Functions----------------------------------------------
    void PureOdometry();
    cv::Mat GetPureOdometry();

    // -------------------------------------------Tracking Functions-------------------------------------------------- 
    void Track();

    // Extract descriptors
    void ExtractKeyPoints(const std::size_t index, cv::Mat polar_image, cv::Mat cart_image, 
                            const ConfigLoader config_loader,
                            std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors);

    // Perform descriptors matching
    void ComputeMatches(std::size_t index, cv::Mat img1, cv::Mat img2, 
                        const std::vector<cv::KeyPoint> keypoints1, const std::vector<cv::KeyPoint> keypoints2,
                        const cv::Mat descriptors1, const cv::Mat descriptors2, 
                        std::vector<cv::DMatch> &matches, const ConfigLoader config_loader);
                        
    // Compute relative pose
    bool ComputeRelativePose(const std::vector<cv::Point2f> prev, const std::vector<cv::Point2f> curr, cv::Mat &RT);

    // Track local map
    bool TrackReferenceKeyFrame();
    bool TrackLocalMap();
    void UpdateLocalMap();
    void UpdateLocalKeyFrames();
    void UpdateLocalPoints();
    void SearchLocalPoints();   
    bool Relocalization(); 
    int GetCurrentFrameInliers();
    void ClearLocalWindow();

    // Constant motion model
    void ConstantMotionModel();
    void ComputeVelocity();

    // Localization
    bool Localised();

    // Save poses
    void StoreNewPose();
    
    // Decide whether to insert new keyframe
    bool NeedNewKeyFrame();

};

// } // namespace ROAM

#endif