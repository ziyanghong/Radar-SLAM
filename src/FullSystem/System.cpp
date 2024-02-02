#include "System.h"

// namespace ROAM{


System::System()
{

}

System::~System()
{

}

System::System(ConfigLoader config_loader):mConfigLoader(config_loader)
{
    // Create map
    mpMap = new Map();

    // Creat Tracking thread
    mpTracker = new Tracking(this, mpMap, config_loader);

    // Create local mapping
    mpLocalMapper= new LocalMapping(mpMap, config_loader);
    // mptLocalMapping = new std::thread(&LocalMapping::Run, mpLocalMapper);

    // Create loop closure
    mpLoopCloser = new LoopClosing(mpMap, config_loader);
    // mptLoopCloser = new std::thread(&LoopClosing::Run, mpLoopCloser);

    // Create Viewer
    mpViewer = new Viewer(this, mpTracker, mpMap, config_loader);
    // mptViewer = new std::thread(&Viewer::Run, mpViewer);

    // Create local FeatureTracker
    if (config_loader.trackMode == 1){
        FeatureTracker *mpFeatureTracker = new FeatureTracker(config_loader);
        mpTracker->SetFeatureTracker(mpFeatureTracker);

        // Viewer set FeatureTracker
        mpViewer->SetFeatureTracker(mpFeatureTracker);
    }

    // Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopCloser(mpLoopCloser);
    mpTracker->SetViewer(mpViewer);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetLocalMapper(mpLocalMapper);   
    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetViewer(mpViewer);

}

void System::SaveMapPoints(){
    std::string mapPointFile = mConfigLoader.dataset_path + "MapPoint.csv";
    ofsMapPoint.open(mapPointFile) ;
    ofsMapPoint << "PointID,x,y" << std::endl;

    std::vector<MapPoint*> vMPs = mpMap->GetAllMapPoints(); 
    for (int i=0 ; i < vMPs.size(); i++)
    {
        MapPoint* pMP = vMPs[i];
        cv::Mat position = pMP->GetWorldPos();

        ofsMapPoint << pMP->mnId << "," 
                    << position.at<float>(0) << "," 
                    << position.at<float>(1) << "," 
                    << std::endl;
    }
}

void System::SaveKeyFramePoses(){

    // For keyframe poses
    ofsKeyFrame.open(mConfigLoader.keyframe_pose_file);
    ofsKeyFrame << "FrameID,tcw_x,tcw_y,tcw_yaw" << std::endl;

    /* Unsorted */
    // std::vector<KeyFrame*> KFs = mpMap->GetAllKeyFrames();
    // for (int i=0 ; i < KFs.size(); i++)
    // {
    //     KeyFrame* pKF = KFs[i];
    //     cv::Mat pose = pKF->GetPose();
    //     double angle_yaw = atan2(pose.at<float>(1, 0), pose.at<float>(0, 0));

    //     ofsKeyFrame << pKF->mnFrameId << "," 
    //                 << pose.at<float>(0, 2) << "," 
    //                 << pose.at<float>(1, 2) << "," 
    //                 << angle_yaw
    //                 << std::endl;
    // }

    /* Sorted */
    std::vector<KeyFrame*> KFs = mpMap->GetAllKeyFrames();
    std::vector<std::vector<double>> vecOFvec;

    // Fill vecOFvec
    for (int i=0 ; i < KFs.size(); i++)
    {
        KeyFrame* pKF = KFs[i];
        cv::Mat pose = pKF->GetPose();
        double angle_yaw = atan2(pose.at<float>(1, 0), pose.at<float>(0, 0));
        std::vector<double> vec = {double(pKF->mnFrameId), pose.at<float>(0, 2), pose.at<float>(1, 2), angle_yaw};
        vecOFvec.push_back(vec);
    }

    // Sort
    std::sort(vecOFvec.begin(), vecOFvec.end(),
          [](const std::vector<double>& a, const std::vector<double>& b){return a[0] < b[0];});

    // Save
    for(int i=0; i < vecOFvec.size(); i++)
    {
        std::vector<double> vec = vecOFvec[i];

        ofsKeyFrame << vec[0] << "," 
                    << vec[1] << "," 
                    << vec[2] << "," 
                    << vec[3]
                    << std::endl;        
    }
}

void System::SaveAllFeaturePoints()
{
    std::string mapPointFile = mConfigLoader.dataset_path + "MapPoint.csv";
    ofsMapPoint.open(mapPointFile) ;
    ofsMapPoint << "PointID,obs,x,y" << std::endl;

    std::vector<KeyFrame*> KFs = mpMap->GetAllKeyFrames();
    std::set<Feature*> Fetures;


    /** Retreive all the features , correct its world position*/
    for (int i = 0; i < KFs.size(); i++)
    {
        KeyFrame *pKFi = KFs[i];
        int frameId = pKFi->mnId;
        std::vector<Feature *> vFeatures = pKFi->mvFeatures;
        std::vector<cv::Point2f> vFeatureCoordinates = pKFi->mvFeatureCoordinates;

        for (int j = 0; j < vFeatures.size(); j++)
        {
            Feature *pFTj = vFeatures[j];
            cv::Point2d image_point = vFeatureCoordinates[j];
            if (!Fetures.count(pFTj))
            {
                cv::Point2d ptLocal = imageFrame2LocalFrame(mConfigLoader.cartImageHeight, mConfigLoader.cartImageWidth,
                                                            mConfigLoader.cartImageScale, image_point);
                cv::Mat LocalPos = cv::Mat(3, 1, CV_32FC1);
                LocalPos.at<float>(0, 0) = ptLocal.x;
                LocalPos.at<float>(1, 0) = ptLocal.y;
                LocalPos.at<float>(2, 0) = 1.0;

                // Compute delta time t between t=0 and the time when landmark is observed
                double theta = atan2(ptLocal.y, ptLocal.x);
                double factorT;
                double factorT0 = 0.5; // The time factor of T(t=0)
                if (theta > 0)
                {
                    factorT = ((M_PI * 2 - theta)) / (2 * M_PI);
                }
                else
                {
                    factorT = (-theta) / (2 * M_PI);
                }
                double frameTimeDiff = pKFi->mTimestamp - pKFi->mPreviousTimestamp;
                double deltaT = frameTimeDiff * (factorT - factorT0);

                // Correct the local observation using optimized velocity
                cv::Mat T0t = cv::Mat::eye(3, 3, CV_32F); // From t=0 to t=t'
                Eigen::Vector3d optimizedVelocity = pKFi->GetVelocity();
                float deltaX = optimizedVelocity(0) * deltaT;
                float deltaY = optimizedVelocity(1) * deltaT;
                float thetaT0t = optimizedVelocity(2) * deltaT;
                T0t.at<float>(0, 0) = cos(thetaT0t);
                T0t.at<float>(0, 1) = -sin(thetaT0t);
                T0t.at<float>(0, 2) = deltaX;
                T0t.at<float>(1, 0) = sin(thetaT0t);
                T0t.at<float>(1, 1) = cos(thetaT0t);
                T0t.at<float>(1, 2) = deltaY;
                cv::Mat correctedLocalPos = T0t * LocalPos;
                cv::Mat WorldPos = pKFi->GetPose() * correctedLocalPos;
                ofsMapPoint << pFTj->mnId << "," 
                            << pFTj->nObs << ","
                            << WorldPos.at<float>(0) << "," 
                            << WorldPos.at<float>(1) << "," 
                            << std::endl;


                Fetures.insert(pFTj);
            }
        }
    }
}

// ToDo, not used ATM
void System::FinalGlobalBA(){
    std::cout << "Start final Global Bundle Adjustment..." << std::endl;
    bool bStopGBA = false;
    int nLoopKF = 0;
    bool bRobust = true;
    Optimizer::GlobalBundleAdjustment(mpMap, 10, &bStopGBA, nLoopKF, bRobust, &mConfigLoader);
}

// } // namespace ROAM
