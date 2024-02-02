#ifndef VIEWER_H
#define VIEWER_H

#include <iostream>
#include <fstream>
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

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "System.h"
#include "Algorithms.h"
#include "FeatureTracker.h"

class Tracking;
class Frame;
class KeyFrame;
class System;
class FeatureTracker;


class Viewer {

public:
    Viewer(System* pSystem, Tracking *pTracking, Map* pMap, const ConfigLoader config_loader);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the radar fps. We use Rviz.
    void Run();

    void UpdateTracking();
    void UpdateMap();
    void UpdateFeatures();

    void CorrectLoop(); // correct the keyframe pose and plot it
    void PlotLoop(KeyFrame* pCurrKF, KeyFrame* pMatchedKF);

    void SetFeatureTracker(FeatureTracker *pFeatureTracker);

private:
    Map* mpMap;
    Tracking* mpTracker;
    FeatureTracker *mpFeatureTracker;    
    ConfigLoader config_loader;
    cv::Mat mCameraPose;
    // ----------------------------------------------Visualization----------------------------------------------------
    // Variables

    // Odometry result txt file
    std::ofstream odometry_result_txt;
    
    // ROS
    int markerId;
    ros::NodeHandle nh;
    tf::TransformBroadcaster broadcaster;
    ros::Publisher pose_pub; // for system pose or mean particles
    ros::Publisher keyframe_pose_pub; // for keyframe poses
    geometry_msgs::PoseStamped pose_msg; // single pose or mean partilce
    geometry_msgs::PoseArray keyframe_poses_msg; // for keyframe poses

    ros::Publisher marker_pub; 
    ros::Publisher marker_pure_odom_pub;
    ros::Publisher marker_covariance_pub;
    ros::Publisher marker_loop_pub;
    visualization_msgs::Marker line_strip;
    visualization_msgs::Marker loop_line;
    visualization_msgs::Marker pure_odom;
    visualization_msgs::Marker covariance;


    image_transport::Publisher keypoint_image_pub;
    image_transport::Publisher keypoint_matches_image_pub;
    ros::Publisher global_point_cloud_pub;
    ros::Publisher reference_point_cloud_pub;
    // Pcl publisher
    ros::Publisher pcl_point_cloud_pub;
    // ros::Publisher frame_point_cloud_pub;

    // Markers
    void publishPose(const cv::Mat Tcw);
    void publishPureOdometry(const cv::Mat Tcw);
    void publishReferenceMapPointCloud(std::vector<MapPoint*> vpMaPoints);
    void publishUpdatedMapPointCloud( std::vector<MapPoint*> vpMaPoints);
    void publishDeletedMapPointCloud(std::vector<MapPoint*> vpMaPoints);
    void publishAllMapPointCloud(std::set<MapPoint*> sMaPoints);

    // Pcl point cloud, not markers
    void publishPointCloud(std::vector<MapPoint*> vpMaPoints);
    // void publishFramePointCloud( cv::Mat Tcw, std::vector<cv::KeyPoint> keypoints);


    
};

#endif