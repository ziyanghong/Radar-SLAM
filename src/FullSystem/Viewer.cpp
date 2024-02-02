#include "Viewer.h"
#include "Tracking.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "LocalMapping.h"

Viewer::Viewer(System *pSystem, Tracking *pTracking, Map *pMap,
               const ConfigLoader _config_loader) : mpTracker(pTracking), mpMap(pMap), config_loader(_config_loader)
{
    markerId = 0;
    // Trajector
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("system_pose", 10);
    pose_msg.header.frame_id = "/world";
    marker_pub = nh.advertise<visualization_msgs::Marker>("path", 10);
    line_strip.scale.x = 2;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.header.frame_id = "/world";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    // Keyframe axis
    keyframe_pose_pub = nh.advertise<geometry_msgs::PoseArray>("keyframe_poses", 10);
    keyframe_poses_msg.header.frame_id = "/world";

    // Loop closure
    marker_loop_pub = nh.advertise<visualization_msgs::Marker>("loop", 10);
    loop_line.scale.x = 4;
    loop_line.type = visualization_msgs::Marker::LINE_STRIP;
    loop_line.header.frame_id = "/world";
    loop_line.action = visualization_msgs::Marker::ADD;
    loop_line.color.g = 1.0;
    loop_line.color.a = 1.0;

    // Pure odometry
    marker_pure_odom_pub = nh.advertise<visualization_msgs::Marker>("pure_odom", 10);
    pure_odom.scale.x = 1;
    pure_odom.type = visualization_msgs::Marker::LINE_STRIP;
    pure_odom.header.frame_id = "/world";
    pure_odom.action = visualization_msgs::Marker::ADD;
    pure_odom.color.r = 1.0;
    pure_odom.color.a = 1.0;

    // Pose uncertainty
    marker_covariance_pub = nh.advertise<visualization_msgs::Marker>("uncertainty", 10);
    covariance.type = visualization_msgs::Marker::CYLINDER;
    covariance.header.frame_id = "/world";
    covariance.color.a = 0.3;
    covariance.color.r = 0.0;
    covariance.color.g = 1.0;
    covariance.color.b = 1.0;

    // Marker Point cloud publisher
    global_point_cloud_pub = nh.advertise<visualization_msgs::Marker>("global_point_cloud", 10);
    reference_point_cloud_pub = nh.advertise<visualization_msgs::Marker>("reference_point_cloud", 10);

    // Pcl point cloud publisher
    pcl_point_cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud", 1);

    // Publish keypoint image
    image_transport::ImageTransport it(nh);
    keypoint_image_pub = it.advertise("keypoint_image", 1);
    keypoint_matches_image_pub = it.advertise("keypoint_matches", 1);

    // Odometry result txt file
    odometry_result_txt.open(_config_loader.odometry_result_file);
    odometry_result_txt << "FrameID,tcw_x,tcw_y,tcw_yaw" << std::endl;
}

void Viewer::Run()
{

    // while (1)
    // {
    // if (mpTracker->mCurrentFrame.IsReady() && mpMap->mvpLocalMapPoints.size() > 0)
    // {
    mCameraPose = mpTracker->mCurrentFrame.GetPose();
    UpdateTracking();
    // }

    // if (mpMap->mvpNewAddedMapPoints.size() > 0){
    //     UpdateMap();
    // }
    if (config_loader.trackMode == 0)
    {
        if (mpMap->MapPointsInMap() > 0)
        {
            UpdateMap();
        }
    }
    else
    {
        // UpdateFeatures(); Shows feature from featureTracker
        // show map point from pMap
        if (mpMap->MapPointsInMap() > 0)
        {
            UpdateMap();
        }
    }

    //     // Sleep
    //     usleep(50000);
    // }
}

void Viewer::SetFeatureTracker(FeatureTracker *pFeatureTracker)
{
    mpFeatureTracker = pFeatureTracker;
}

/* To visualize features */
void Viewer::UpdateFeatures()
{
    // Plot all new features
    // std::set<Feature *> spNewFeatures = mpFeatureTracker->GetNewFeatures();
    // for (std::set<Feature *>::const_iterator sit = spNewFeatures.begin(), send = spNewFeatures.end(); sit != send; sit++)
    // {
    //     Feature *pFT = *sit;

    //     // MapPoint
    //     visualization_msgs::Marker map_point;
    //     map_point.lifetime = ros::Duration();
    //     map_point.header.stamp = ros::Time::now();
    //     map_point.scale.x = config_loader.pointCloudSize;
    //     map_point.scale.y = config_loader.pointCloudSize;
    //     map_point.type = visualization_msgs::Marker::CUBE;
    //     map_point.header.frame_id = "/world";
    //     map_point.action = visualization_msgs::Marker::ADD;
    //     map_point.ns = "map_point";
    //     map_point.id = pFT->mnId;
    //     map_point.color.b = 1.0;
    //     map_point.color.g = 1.0;
    //     map_point.color.r = 1.0;
    //     map_point.color.a = 1.0;
    //     cv::Mat WorldPos = pFT->mWorldPos;
    //     map_point.pose.position.x = WorldPos.at<float>(0, 0);
    //     map_point.pose.position.y = WorldPos.at<float>(1, 0);
    //     map_point.pose.position.z = 0.0;
    //     global_point_cloud_pub.publish(map_point);
    // }

    // Plot the tracked features
    std::vector<Feature *> vpTrackedFeatures = mpFeatureTracker->GetTrackedFeatures();
    for (int i = 0; i < vpTrackedFeatures.size(); i++)
    {
        Feature *pFT = vpTrackedFeatures[i];

        // MapPoint
        visualization_msgs::Marker map_point;
        map_point.lifetime = ros::Duration();
        map_point.header.stamp = ros::Time::now();
        map_point.scale.x = config_loader.pointCloudSize;
        map_point.scale.y = config_loader.pointCloudSize;
        map_point.type = visualization_msgs::Marker::CUBE;
        map_point.header.frame_id = "/world";
        map_point.action = visualization_msgs::Marker::ADD;
        map_point.ns = "map_point";
        map_point.id = pFT->mnId;
        map_point.color.b = 1.0;
        map_point.color.g = 1.0;
        map_point.color.r = 1.0;
        map_point.color.a = 1.0;
        cv::Mat WorldPos = pFT->mWorldPos;
        map_point.pose.position.x = WorldPos.at<float>(0, 0);
        map_point.pose.position.y = WorldPos.at<float>(1, 0);
        map_point.pose.position.z = 0.0;
        global_point_cloud_pub.publish(map_point);
    }
}

void Viewer::UpdateTracking()
{
    std::unique_lock<std::mutex> lock(mpMap->mMutexMapUpdate);

    // Visualize keypoint image
    cv::Mat currentKeyPointImage = mpTracker->mCurrentFrame.mKeyPointsImage;
    cv::Mat text_img(currentKeyPointImage.rows, 400, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat out;
    cv::hconcat(text_img, currentKeyPointImage, out);

    // Frame string
    std::string CurrentFramestring = "Frame ID: " + std::to_string(mpTracker->mCurrentFrame.mnId);
    cv::putText(out,
                CurrentFramestring,
                cv::Point(20, 60),              // Coordinates
                cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                2.0,                            // Scale. 2.0 = 2x bigger
                cv::Scalar(255, 255, 255),      // BGR Color
                2);                             // Line Thickness (Optional)

    // Reference mappoint number
    std::string FrameKPstring = "FrameKPs: " + std::to_string(mpTracker->mCurrentFrame.N);
    cv::putText(out,
                FrameKPstring,
                cv::Point(20, 120),             // Coordinates
                cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                2.0,                            // Scale. 2.0 = 2x bigger
                cv::Scalar(255, 255, 255),      // BGR Color
                2);

    // // Reference mappoint number
    // std::string RefMPstring = "RefMPs: " + std::to_string(mpTracker->mCurrentFrame.nMatches);
    // cv::putText(out,
    //             RefMPstring,
    //             cv::Point(20, 180),             // Coordinates
    //             cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
    //             2.0,                            // Scale. 2.0 = 2x bigger
    //             cv::Scalar(255, 255, 255),      // BGR Color
    //             2);

    // KF string
    std::string KFstring = "KFs: " + std::to_string(mpMap->KeyFramesInMap());
    cv::putText(out,
                KFstring,
                cv::Point(20, 240),             // Coordinates
                cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                2.0,                            // Scale. 2.0 = 2x bigger
                cv::Scalar(255, 255, 255),      // BGR Color
                2);                             // Line Thickness (Optional)

    // MapPoint string
    std::string MPstring = "MPs: " + std::to_string(mpMap->MapPointsInMap());
    cv::putText(out,
                MPstring,
                cv::Point(20, 300),             // Coordinates
                cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                2.0,                            // Scale. 2.0 = 2x bigger
                cv::Scalar(255, 255, 255),      // BGR Color
                2);                             // Line Thickness (Optional)

    // Inliers string
    std::string InliersString = "Inliers: " + std::to_string(mpTracker->GetCurrentFrameInliers());
    if (mpTracker->GetCurrentFrameInliers() >= 5)
    {
        cv::putText(out,
                    InliersString,
                    cv::Point(20, 360),             // Coordinates
                    cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                    2.0,                            // Scale. 2.0 = 2x bigger
                    cv::Scalar(0, 255, 0),          // BGR Color
                    2);                             // Line Thickness (Optional)
    }
    else
    {
        cv::putText(out,
                    InliersString,
                    cv::Point(20, 360),             // Coordinates
                    cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
                    2.0,                            // Scale. 2.0 = 2x bigger
                    cv::Scalar(0, 0, 255),          // BGR Color
                    2);                             // Line Thickness (Optional)
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", out).toImageMsg();
    keypoint_image_pub.publish(msg);

    // Publish keypoint matches image
    sensor_msgs::ImagePtr msg_matches = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mpTracker->mCurrentFrame.mImageMatches).toImageMsg();
    keypoint_matches_image_pub.publish(msg_matches);

    // Publish odometry and mapping pose
    cv::Mat currentPose = mpTracker->mCurrentFrame.GetPose();
    int mnId = mpTracker->mCurrentFrame.mnId;
    publishPose(currentPose);
    // Save tracking pose
    double tangent = currentPose.at<float>(1, 0) / currentPose.at<float>(0, 0);
    double angle_yaw = atan2(currentPose.at<float>(1, 0), currentPose.at<float>(0, 0));
    odometry_result_txt << mnId << ","
                        << currentPose.at<float>(0, 2) << ","
                        << currentPose.at<float>(1, 2) << ","
                        << angle_yaw
                        << std::endl;

    // Publish covariance
    covariance.pose.position.x = currentPose.at<float>(0, 2);
    covariance.pose.position.y = currentPose.at<float>(1, 2);
    covariance.pose.position.z = 0.0;
    covariance.pose.orientation.x = 0;
    covariance.pose.orientation.y = 0;
    covariance.pose.orientation.z = 0;
    covariance.pose.orientation.w = 1;
    covariance.scale.x = mpTracker->Pk.at<float>(0, 0);
    covariance.scale.y = mpTracker->Pk.at<float>(1, 1);
    covariance.scale.z = 0.01;
    marker_covariance_pub.publish(covariance);

    // std::cout << "Publish pose." << std::endl;
    // std::cout << currentPose << std::endl;

    // Publishe pure odometry pose
    cv::Mat pureOdometryPose = mpTracker->GetPureOdometry();
    publishPureOdometry(pureOdometryPose);

    // Publish key frames
    std::vector<KeyFrame *> KFs = mpMap->GetAllKeyFrames();
    keyframe_poses_msg.poses.clear(); // clear the historic trajectory

    std::vector<std::vector<double>> Matrix;
    for (int i = 0; i < KFs.size(); i++)
    {
        KeyFrame *pKF = KFs[i];
        cv::Mat pose = pKF->GetPose();

        tf2::Quaternion q;
        double tangent = pose.at<float>(1, 0) / pose.at<float>(0, 0);
        double angle_yaw = atan2(pose.at<float>(1, 0), pose.at<float>(0, 0));
        q.setRPY(0.0, 0.0, angle_yaw);
        q.normalize();

        float x = pose.at<float>(0, 2);
        float y = pose.at<float>(1, 2);

        geometry_msgs::Pose keyframe_msg;
        keyframe_msg.position.x = x;
        keyframe_msg.position.y = y;
        keyframe_msg.position.z = 0.0;

        keyframe_msg.orientation.x = q.x();
        keyframe_msg.orientation.y = q.y();
        keyframe_msg.orientation.z = q.z();
        keyframe_msg.orientation.w = q.w();
        keyframe_poses_msg.poses.push_back(keyframe_msg);
    }
    keyframe_pose_pub.publish(keyframe_poses_msg);

    if (config_loader.trackMode == 0)
    {
        // Visualize reference map points
        publishReferenceMapPointCloud(mpMap->mvpLocalMapPoints);
        mpMap->mvpLocalMapPoints.clear();
    }

    // // Write to txt file
    // std::ofstream myfile;
    // std::string frame_id = std::to_string(mpTracker->mCurrentFrame.mnId);
    // std::string log_file_str = config_loader.log_path + std::string(6-frame_id.length(), '0').append(frame_id) + ".txt";
    // myfile.open(log_file_str);
    // myfile << InliersString << std::endl;

    // myfile << "Keyframe based odometry: " << std::endl;
    // for(int i=0; i<currentPose.rows; i++)
    // {
    //     for(int j=0; j<currentPose.cols; j++)
    //     {
    //         myfile<<currentPose.at<float>(i,j)<<",";
    //     }
    //     myfile<<std::endl;
    // }

    // myfile << "Pure odometry: " << std::endl;
    // for(int i=0; i<pureOdometryPose.rows; i++)
    // {
    //     for(int j=0; j<pureOdometryPose.cols; j++)
    //     {
    //         myfile<<pureOdometryPose.at<float>(i,j)<<",";
    //     }
    //     myfile<<std::endl;
    // }

    // myfile.close();
}

void Viewer::UpdateMap()
{
    std::unique_lock<std::mutex> lock(mpMap->mMutexMapUpdate);

    // Visualize new added map point
    // publishUpdatedMapPointCloud mpMap->mvpUpdatedMapPoints);
    // publishPointCloud(mpMap->mvpUpdatedMapPoints);
    mpMap->mvpUpdatedMapPoints.clear();

    // Delete map point
    // publishDeletedMapPointCloud( mpMap->mvDeleteMapPointQueue);
    mpMap->mvDeleteMapPointQueue.clear();

    // Visualize all map points get updated
    // publishAllMapPointCloud( mpMap->GetAllMapPoints());
    publishPointCloud(mpMap->GetAllMapPoints());
}

void Viewer::PlotLoop(KeyFrame *pCurrKF, KeyFrame *pMatchedKF)
{

    cv::Mat pose = pCurrKF->GetPose();
    geometry_msgs::Point pCurr;
    pCurr.x = pose.at<float>(0, 2);
    pCurr.y = pose.at<float>(1, 2);
    pCurr.z = 0.0;
    loop_line.points.push_back(pCurr);

    pose = pMatchedKF->GetPose();
    geometry_msgs::Point pMatched;
    pMatched.x = pose.at<float>(0, 2);
    pMatched.y = pose.at<float>(1, 2);
    pMatched.z = 0.0;
    loop_line.points.push_back(pMatched);

    marker_loop_pub.publish(loop_line);
    // ros::Duration(2).sleep();
    loop_line.points.clear();
    marker_loop_pub.publish(loop_line);
}

void Viewer::CorrectLoop()
{
    std::vector<KeyFrame *> KFs = mpMap->GetAllKeyFrames();
    line_strip.points.clear(); // clear the historic trajectory

    std::vector<std::vector<double>> Matrix;
    for (int i = 0; i < KFs.size(); i++)
    {
        std::vector<double> row;
        KeyFrame *pKF = KFs[i];
        cv::Mat pose = pKF->GetPose();
        row.push_back(pKF->mnFrameId);
        row.push_back(pose.at<float>(0, 2));
        row.push_back(pose.at<float>(1, 2));
        Matrix.push_back(row);
    }

    // Sort based on first column
    sortrows(Matrix, 0);

    for (int i = KFs.size() - 1; i > 0; i--)
    {
        std::vector<double> row = Matrix[i];
        geometry_msgs::Point p;
        p.x = row[1];
        p.y = row[2];
        p.z = 0.0;

        line_strip.points.push_back(p);

        // std::cout << "KF id: " << row[0] << " x: " << p.x << " y: " << p.y << std::endl;
    }

    marker_pub.publish(line_strip);
}

void Viewer::publishPose(const cv::Mat Tcw)
{
    /*
     * std::atan2 allows calculating the arctangent of all four quadrants. std::atan only allows calculating from quadrants 1 and 4.  
     */

    // Current pose
    tf2::Quaternion q;
    double tangent = Tcw.at<float>(1, 0) / Tcw.at<float>(0, 0);
    double angle_yaw = atan2(Tcw.at<float>(1, 0), Tcw.at<float>(0, 0));
    q.setRPY(0.0, 0.0, angle_yaw);
    q.normalize();

    float x = Tcw.at<float>(0, 2);
    float y = Tcw.at<float>(1, 2);
    // std::cout << "Published pose Tcw x: " << Tcw.at<float>(0, 2) << " Tcw y: " << Tcw.at<float>(1, 2) << " Tcw yaw: " << angle_yaw << std::endl;

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = 0.0;

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;

    line_strip.points.push_back(p);

    // Send TF
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(Tcw.at<float>(0, 2), Tcw.at<float>(1, 2), 0.0));
    tf::Quaternion q_tf;
    q_tf.setRPY(0.0, 0.0, angle_yaw);
    transform.setRotation(q_tf);
    broadcaster.sendTransform(
        tf::StampedTransform(transform,
                             ros::Time::now(), "world", "base_link"));

    // Publish
    pose_pub.publish(pose_msg);
    marker_pub.publish(line_strip);
}

void Viewer::publishPureOdometry(const cv::Mat Tcw)
{
    /*
     * std::atan2 allows calculating the arctangent of all four quadrants. std::atan only allows calculating from quadrants 1 and 4.  
     */

    // Current pose
    tf2::Quaternion q;
    double tangent = Tcw.at<float>(1, 0) / Tcw.at<float>(0, 0);
    double angle_yaw = atan2(Tcw.at<float>(1, 0), Tcw.at<float>(0, 0));
    q.setRPY(0.0, 0.0, angle_yaw);
    q.normalize();

    float x = Tcw.at<float>(0, 2);
    float y = Tcw.at<float>(1, 2);
    // std::cout << "Published pose Tcw x: " << Tcw.at<float>(0, 2) << " Tcw y: " << Tcw.at<float>(1, 2) << " Tcw yaw: " << angle_yaw << std::endl;

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = 0.0;

    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;

    pure_odom.points.push_back(p);

    // Send TF
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(Tcw.at<float>(0, 2), Tcw.at<float>(1, 2), 0.0));
    tf::Quaternion q_tf;
    q_tf.setRPY(0.0, 0.0, angle_yaw);
    transform.setRotation(q_tf);
    broadcaster.sendTransform(
        tf::StampedTransform(transform,
                             ros::Time::now(), "world", "base_link"));

    // Publish
    marker_pure_odom_pub.publish(pure_odom);
}

void Viewer::publishReferenceMapPointCloud(std::vector<MapPoint *> vpMaPoints)
{
    std::cout << "Viewer::Published " << vpMaPoints.size() << " reference map point." << std::endl;
    // Global point cloud
    visualization_msgs::Marker reference_points;
    reference_points.lifetime = ros::Duration();
    reference_points.header.stamp = ros::Time::now();
    reference_points.scale.x = config_loader.pointCloudSize;
    reference_points.scale.y = config_loader.pointCloudSize;
    reference_points.type = visualization_msgs::Marker::POINTS;
    reference_points.header.frame_id = "/world";
    reference_points.action = visualization_msgs::Marker::ADD;
    reference_points.id = 1;
    reference_points.ns = "reference_points";
    reference_points.color.g = 1.0;
    reference_points.color.a = 1.0;

    for (int i = 0; i < vpMaPoints.size(); i++)
    {
        MapPoint *pMP = vpMaPoints[i];
        cv::Mat WorldPos = pMP->GetWorldPos();
        geometry_msgs::Point p;
        p.x = WorldPos.at<float>(0, 0);
        // std::cout << "publishNewMapPointCloud::p.x: " << p.x << std::endl;

        p.y = WorldPos.at<float>(1, 0);
        // std::cout << "publishNewMapPointCloud::p.y: " << p.y << std::endl;

        p.z = 0.0;
        reference_points.points.push_back(p);
    }

    // publish
    reference_point_cloud_pub.publish(reference_points);
}

void Viewer::publishUpdatedMapPointCloud(std::vector<MapPoint *> vpMaPoints)
{
    // std::cout << "Viewer::Published " <<  vpMaPoints.size() << " global map point." << std::endl;

    for (std::vector<MapPoint *>::const_iterator it = vpMaPoints.begin(), itend = vpMaPoints.end(); it != itend; it++)
    {
        MapPoint *pMP = *it;
        if (pMP->mbUpdated)
        {
            pMP->mbUpdated = false;

            // marker
            visualization_msgs::Marker map_point;
            map_point.lifetime = ros::Duration();
            map_point.header.stamp = ros::Time::now();
            map_point.scale.x = config_loader.pointCloudSize;
            map_point.scale.y = config_loader.pointCloudSize;
            map_point.type = visualization_msgs::Marker::CUBE;
            map_point.header.frame_id = "/world";
            map_point.action = visualization_msgs::Marker::ADD;
            map_point.ns = "map_point";
            map_point.id = pMP->mnId;
            map_point.color.b = 1.0;
            map_point.color.g = 1.0;
            map_point.color.r = 1.0;
            map_point.color.a = 1.0;
            cv::Mat WorldPos = pMP->GetWorldPos();
            map_point.pose.position.x = WorldPos.at<float>(0, 0);
            map_point.pose.position.y = WorldPos.at<float>(1, 0);
            map_point.pose.position.z = 0.0;
            global_point_cloud_pub.publish(map_point);
        }
    }
}

void Viewer::publishDeletedMapPointCloud(std::vector<MapPoint *> vpMaPoints)
{

    for (std::vector<MapPoint *>::const_iterator it = vpMaPoints.begin(), itend = vpMaPoints.end(); it != itend; it++)
    {
        MapPoint *pMP = *it;
        // marker
        visualization_msgs::Marker map_point;
        map_point.lifetime = ros::Duration();
        map_point.header.stamp = ros::Time::now();
        map_point.scale.x = config_loader.pointCloudSize;
        map_point.scale.y = config_loader.pointCloudSize;
        map_point.type = visualization_msgs::Marker::CUBE;
        map_point.header.frame_id = "/world";
        map_point.action = visualization_msgs::Marker::DELETE;
        map_point.ns = "map_point";
        map_point.id = pMP->mnId;

        global_point_cloud_pub.publish(map_point);
    }
}

void Viewer::publishAllMapPointCloud(std::set<MapPoint *> sMaPoints)
{

    for (std::set<MapPoint *>::const_iterator it = sMaPoints.begin(), itend = sMaPoints.end(); it != itend; it++)
    {
        MapPoint *pMP = *it;
        if (pMP->mbUpdated)
        {
            // MapPoint
            visualization_msgs::Marker map_point;
            map_point.lifetime = ros::Duration();
            map_point.header.stamp = ros::Time::now();
            map_point.scale.x = config_loader.pointCloudSize;
            map_point.scale.y = config_loader.pointCloudSize;
            map_point.type = visualization_msgs::Marker::CUBE;
            map_point.header.frame_id = "/world";
            map_point.action = visualization_msgs::Marker::ADD;
            map_point.ns = "map_point";
            map_point.id = pMP->mnId;
            map_point.color.b = 1.0;
            map_point.color.g = 1.0;
            map_point.color.r = 1.0;
            map_point.color.a = 1.0;
            cv::Mat WorldPos = pMP->GetWorldPos();
            map_point.pose.position.x = WorldPos.at<float>(0, 0);
            map_point.pose.position.y = WorldPos.at<float>(1, 0);
            map_point.pose.position.z = 0.0;
            global_point_cloud_pub.publish(map_point);
        }
    }
}

void Viewer::publishPointCloud(std::vector<MapPoint *> vpMaPoints)
{
    int downsample_rate = 3;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointer_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pointer_point_cloud->header.frame_id = "/world";
    pointer_point_cloud->height = 1;
    pointer_point_cloud->width = vpMaPoints.size() / downsample_rate;

    for (int i = 0; i < vpMaPoints.size(); i++)
    {
        MapPoint *pMP = vpMaPoints[i];
        if (pMP && i % downsample_rate == 0)
        {
            cv::Mat WorldPos = pMP->GetWorldPos();
            pointer_point_cloud->points.push_back(pcl::PointXYZ(WorldPos.at<float>(0, 0), WorldPos.at<float>(1, 0), 0.0));
        }
    }
    pcl_point_cloud_pub.publish(pointer_point_cloud);
}

// void Viewer::publishFramePointCloud(cv::Mat Tcw, std::vector<cv::KeyPoint> keypoints){
//     pcl::PointCloud<pcl::PointXYZ>::Ptr pointer_point_cloud  (new pcl::PointCloud<pcl::PointXYZ>);
//     pointer_point_cloud->header.frame_id = "/base_link";
//     pointer_point_cloud->height = 1;
//     pointer_point_cloud->width = keypoints.size();

//     for (int i=0;i<keypoints.size(); i++)
//     {
//         cv::Point2d pt;
//         pt.x = keypoints[i].pt.x;
//         pt.y = keypoints[i].pt.y;
//         pt = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth, config_loader.cartImageScale, pt);
//         pointer_point_cloud->points.push_back(pcl::PointXYZ(pt.x, pt.y, 0.0));
//     }

//     tf::Transform transform;
//     transform.setOrigin(tf::Vector3(Tcw.at<float>(0, 2), Tcw.at<float>(1, 2), 0.0));
//     tf::Quaternion q;
//     q.setRPY(0.0, 0.0, atan(Tcw.at<float>(1, 0) / Tcw.at<float>(0, 0)));
//     transform.setRotation( q );
//     broadcaster.sendTransform(
//                     tf::StampedTransform(transform,
//                     ros::Time::now(),"world", "base_link"));
//     frame_point_cloud_pub.publish(pointer_point_cloud);
// }
