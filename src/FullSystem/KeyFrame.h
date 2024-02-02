#ifndef KEYFRAME_H
#define KEYFRAME_H

// FLIRT
// #include <feature/Detector.h>
// #include <feature/ShapeContext.h>
// #include <feature/BetaGrid.h>
// #include <feature/RangeDetector.h>
// #include <feature/CurvatureDetector.h>
// #include <feature/NormalBlobDetector.h>
// #include <feature/NormalEdgeDetector.h>
// #include <feature/RansacFeatureSetMatcher.h>
// #include <feature/RansacMultiFeatureSetMatcher.h>
// #include <sensorstream/CarmenLog.h>
// #include <sensorstream/LogSensorStream.h>
// #include <sensorstream/SensorStream.h>
// #include <utils/SimpleMinMaxPeakFinder.h>
// #include <utils/HistogramDistances.h>

#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
#include <utility>

#include <set>
#include <mutex>
#include <iterator> 

#include "MapPoint.h"
#include "Frame.h"
#include "Map.h"
#include "Converter.h"
#include "Feature.h"
// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
// Global feature generation
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "generateGlobalFeature_types.h"
#include "generateGlobalFeature.h"
#include "generateGlobalFeature_emxAPI.h"
#include "generateGlobalFeature_terminate.h"
#include "rt_nonfinite.h"

// namespace ROAM
// {
    
class Frame;
class MapPoint;
class Map;
class Feature;
class KeyFrame 
{
public:
    // ------------------------------------------- Functions-------------------------------------------------- 
    KeyFrame();
    KeyFrame(Frame F, Map* pMap);
    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    void SetM2dpFeature(const cv::Mat &Feature);
    void SetPointcloud(const cv::Mat &Pointcloud);

    Eigen::Matrix<double, 3, 1> GetVelocity();    
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();
    int GetNumberMatchesWithLastKeyFrame();
    int GetNumberMatchesWithLoop();
    void SetNumberMatchesWithLoop(int n);

    // Map points functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    std::vector<MapPoint*> GetMapPointMatches();   

    // Covisibility graph functions
    void UpdateConnections();
    std::vector<KeyFrame*> GetVectorCovisibleKeyFrames();

    // Loop closing
    void AddLoopEdge(KeyFrame *pKF);
    std::set<KeyFrame*> GetLoopEdges();
    cv::Mat GetFeature();
    cv::Mat GetFeatureReverse();

    // Spanning tree functions
    KeyFrame* GetParent();
    void ChangeParent(KeyFrame* pKF);
    void AddChild(KeyFrame *pKF);

    // Descriptors and interest points
    // std::vector<InterestPoint *> GetInterestPoints(); // FLIRT
    void ComputeGlobalDescriptor(); // M2DP

    bool isBad();

    // ------------------------------------------- Variables-------------------------------------------------- 
    static long unsigned int nNextId;
    long unsigned int mnId;
    long unsigned int mnFrameId;
    double mTimestamp;   
    cv::Mat mCartImage;
    cv::Mat mPolarImage;
    // cv::Mat mCartImage;    
    cv::Mat mvDescriptors;
    std::vector<cv::KeyPoint> mvKeypoints;    
    cv::Mat  mvPointCloud;
    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    // Feature Tracker variables
    std::vector<cv::Point2f> mvFeatureCoordinates;
    std::vector<Feature*> mvFeatures;   
    double mPreviousTimestamp;
    // Velocity
    Eigen::Matrix<double, 3, 1> mVelocity;


    // number of matches
    int mnMatchesToLoop;
    int mnMatchesWithLastKeyFrame;
    
    // Config loader
    ConfigLoader mConfigLoader;          
protected:
    // ------------------------------------------- Functions-------------------------------------------------- 
    // void DetectInterestPoints(); // FLIRT
    // void DescribeInterestPoints(); // FLIRT


    // ------------------------------------------- Variables-------------------------------------------------- 
    // cv::Mat mPolarImage;
    cv::Mat mFeature;
    cv::Mat mFeatureReverse; 

    // SE2 Pose and Inverse Pose
    cv::Mat Tcw;
    cv::Mat Twc;

    // Pose Uncertainty
    cv::Mat Pk;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent = NULL; // Parent KeyFrame
    std::set<KeyFrame*> mspChildrens; // Children KeyFrames
    std::set<KeyFrame*> mspLoopEdges;

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    // All connected keyframes
    std::vector<KeyFrame*> mvpConnectedKeyFrames;

    // FLIRT
    // LaserReading* mpLaserReading;
    // std::vector<InterestPoint *> mvpInterestPoints;  

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;  

    // Map
    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;   
};

// } // namespace ROAM

#endif