#include "Tracking.h"
// Optimizer
#include "Optimizer.h"

// namespace ROAM{

Tracking::Tracking(System *pSystem, Map *pMap, const ConfigLoader _ConfigLoader) : mpSystem(pSystem),
                                                                                   mpMap(pMap),
                                                                                   mbLocalised(false),
                                                                                   mConfigLoader(_ConfigLoader)
{
    // Iniial pose
    Tcw_init = cv::Mat::eye(3, 3, CV_32F);

    // Initial Covariance
    Pk = cv::Mat::zeros(3, 3, CV_32F);

    // // Set noise matrix Qk
    // Qk = cv::Mat::eye(3,3, CV_32F);
    // Qk.at<float>(0,0) = 0.03;
    // Qk.at<float>(1,1) = 0.03;
    // Qk.at<float>(2,2) = 0.01;
    processNoises[0] = 0.03; // x
    processNoises[1] = 0.03; // y
    processNoises[2] = 0.03; // theta
}

Tracking::~Tracking()
{
}

void Tracking::GrabSensorInput(const std::size_t &frame_id, const double timestamp,
                               const cv::Mat &polar_image, const cv::Mat &cart_image)
{
    // auto start = std::chrono::high_resolution_clock::now();

    // printf("Grabbed new sensor signal\n");
    if (frame_id == 1)
        mPreviousTimestamp = timestamp;

    mnId = frame_id;
    mCurrentTimestamp = timestamp;
    mCurrentFrame = Frame(frame_id, timestamp, mPreviousTimestamp, polar_image, cart_image, mConfigLoader);

    Track();
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // std::cout << "Tracking computation time is: " << duration.count() << " milliseconds." << std::endl;

    mpViewer->Run();

    // start = std::chrono::high_resolution_clock::now();
    mpLocalMapper->Run();
    // stop = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // std::cout << "LocalMapping computation time is: " << duration.count() << " milliseconds." << std::endl;

    /* Sequentially running loopcloser */
    // start = std::chrono::high_resolution_clock::now();
    if (mConfigLoader.loopClosure)
    {
        mpLoopCloser->Run();
    }
    // stop = std::chrono::high_resolution_clock::now();
    // duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // std::cout << "LoopClosing computation time is: " << duration.count() << " milliseconds." << std::endl;

    /* Pure odometry between two frames*/
    // PureOdometry();
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // std::cout << "Each frame computation time is: " << duration.count() << " milliseconds." << std::endl;
}

int Tracking::GetTrackingState()
{
    return mState;
}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void Tracking::SetFeatureTracker(FeatureTracker *pFeatureTracker)
{
    mpFeatureTracker = pFeatureTracker;
}

void Tracking::SetLoopCloser(LoopClosing *pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer = pViewer;
}

void Tracking::SetLocalised()
{
    mbLocalised = true;
}

bool Tracking::Localised()
{
    if (!mbLocalised)
        return false;
}

void Tracking::PureOdometry()
{
    if (!mbPureOdomInitilized)
    {
        // Initialization
        mbPureOdomInitilized = true;
    }
    else
    {
        std::size_t id = mCurrentFrame.mnId;
        cv::Mat img1 = mCurrentFrame.mCartImage;
        cv::Mat img2 = mPreviousFrame.mCartImage;
        std::vector<cv::KeyPoint> keypoints1 = mCurrentFrame.mvKeypoints;
        std::vector<cv::KeyPoint> keypoints2 = mPreviousFrame.mvKeypoints;
        cv::Mat descriptors1 = mCurrentFrame.mvDescriptors;
        cv::Mat descriptors2 = mPreviousFrame.mvDescriptors;

        // Compute matches
        std::vector<cv::DMatch> matches;
        ComputeMatches(id, img1, img2, keypoints1, keypoints2, descriptors1, descriptors2,
                       matches, mConfigLoader);

        std::vector<cv::Point2f> prev, curr;
        for (int i = 0; i < matches.size(); i++)
        {
            //-- Get the keypoints from the matches
            curr.push_back(keypoints1[matches[i].queryIdx].pt);
            prev.push_back(keypoints2[matches[i].trainIdx].pt);
        }

        // Compute relative motion
        cv::Mat RT_SVD;
        bool bTrackOK = ComputeRelativePose(prev, curr, RT_SVD);

        // Optimize relative transformation using the RT estimated by SVD
        Optimizer *pOptmizer = new Optimizer();
        cv::Mat RT_Optimized = pOptmizer->OptimizeRelativePose(prev, curr, RT_SVD, mConfigLoader);
        delete pOptmizer;
        Tcw_pure = Tcw_pure * RT_Optimized;
    }

    mPreviousFrame = mCurrentFrame;
}

cv::Mat Tracking::GetPureOdometry()
{
    return Tcw_pure.clone();
}

std::vector<Frame *> Tracking::GetLocalWindowFrames()
{
    // Retreive the local frames pointer
    std::vector<Frame *> vpWindowFrames;
    for (int i = 0; i < mvWindowFrames.size(); i++)
    {
        Frame *pFi = &mvWindowFrames[i];
        vpWindowFrames.push_back(pFi);
    }
    return vpWindowFrames;
}

// Odometry and local mapping
void Tracking::Track()
{
    // std::cout << "Enter track().\n";
    // std::cout << "Current number of KeyFrame is: " << mpMap->KeyFramesInMap() << std::endl;
    // std::cout << "Current number of MapPoint is: " << mpMap->MapPointsInMap() << std::endl;

    // std::cout << "mState: " << mState << std::endl;

    if (mState == NOT_INITIALIZED)
    {
        // Initialize system
        Initialization();
    }
    else
    {
        // System is initialized. Track Frame.
        bool bTrackOK = 0;

        // If tracking is not lost
        if (mState == OK)
        {
            bTrackOK = TrackReferenceKeyFrame(); // In journal version, this is not used

            // Now we have initial estimation of the vehicle pose and matching. Track the local map
            if (bTrackOK)
            {
                bTrackOK = TrackLocalMap();  // In journal version, this is used to compute pose
            }

            if (!bTrackOK)
            {
                ConstantMotionModel();
            };

            // // Update covariance
            // PredictCovariance(); // Not used

            // Check if we need to insert a new keyframe
            if (NeedNewKeyFrame())
            {
                CreateNewKeyFrame();
            }

        }
        else
        {
            // ToDo: Relocalization
            bTrackOK = Relocalization();
        }

        // Compute velocity
        ComputeVelocity();

        // Should not be here
        // Move it to somewhere else in the future.
        mState = OK;
    }

    mCurrentFrame.mbReady = true;
    mPreviousFrame = mCurrentFrame;

    // Store the pose of current frame
    StoreNewPose();
    // std::cout << "Exit from track()." << std::endl;
}
void Tracking::Initialization()
{

    Tcw_init.convertTo(Tcw_init, CV_32F);

    // Set Pose
    mCurrentFrame.SetPose(Tcw_init);

    // ---------------------------------------Journal added section------------------------------------------
    bool initlized = true;
    if (mConfigLoader.trackMode == 1)
    {
        initlized = mpFeatureTracker->Initialized(&mCurrentFrame, mConfigLoader);
    }

    if (initlized)
    {
        mvWindowFrames.push_back(mCurrentFrame);
        // cv::Mat CurrentFeature = mCurrentFrame.GetFeature();
        // std::cout << "CurrentFeature rows: " << CurrentFeature.rows << " CurrentFeature cols: " << CurrentFeature.cols << std::endl;


        // ---------------------------------------Original code snippet--------------------------------------------

        KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap);

        // Set Current KeyFrame
        mpCurrentKeyFrame = pKFini;

        // Add all the keypoints initially as map points
        for (int i = 0; i < mCurrentFrame.mvKeypoints.size(); i++)
        {
            cv::Point2f pt = imageFrame2LocalFrame(mConfigLoader.cartImageHeight, mConfigLoader.cartImageWidth,
                                                   mConfigLoader.cartImageScale, mCurrentFrame.mvKeypoints[i].pt);
            cv::Mat WorldPos = cv::Mat(3, 1, CV_32FC1);
            WorldPos.at<float>(0, 0) = pt.x;
            WorldPos.at<float>(1, 0) = pt.y;
            WorldPos.at<float>(2, 0) = 1.0;

            MapPoint *pMP = new MapPoint(WorldPos, pKFini, mpMap);
            pMP->AddObservation(pKFini, i);
            pMP->SetDescriptor(mCurrentFrame.mvDescriptors.row(i));

            // Add map point to the keyframe and map
            pKFini->AddMapPoint(pMP, i);
            mpMap->AddMapPoint(pMP);
        }

        // Add first key frame to map
        mpMap->AddOriginKeyFrame(pKFini);

        // Add keyframe to local mapper
        mpLocalMapper->InsertKeyFrame(pKFini);

        // Set tracking state
        mState = OK;

        std::cout << "Tracking is initilized." << std::endl;
    }
    else
    {
        // Set tracking state
        mState = NOT_INITIALIZED;
    }
}

void Tracking::PredictCovariance()
{
    double Xk[3];
    cv::Mat pose = mCurrentFrame.GetPose();

    Xk[0] = pose.at<float>(0, 2);
    Xk[1] = pose.at<float>(1.2);
    Xk[2] = atan2(pose.at<float>(1, 0), pose.at<float>(0, 0));
    // std::cout << "Xk[0]: " << Xk[0] << " Xk[1]: " << Xk[1] << " Xk[2]:" << Xk[2] << std::endl;

    double Uk[3];
    cv::Mat pose_prev = mPreviousFrame.GetPoseInverse();
    // std::cout << "previous inverse pose:" << std::endl << pose_prev << std::endl;
    pose_prev.convertTo(pose_prev, CV_32F);

    cv::Mat RT = pose_prev * pose;
    // std::cout << "RT pose:" << std::endl << RT << std::endl;

    Uk[0] = RT.at<float>(0, 2);
    Uk[1] = RT.at<float>(1, 2);
    Uk[2] = atan2(RT.at<float>(1, 0), RT.at<float>(0, 0));
    // std::cout << "Uk[0]: " << Uk[0] << " Uk[1]: " << Uk[1] << " Uk[2]:" << Uk[2] << std::endl;

    Pk = updateCovariance(Pk, Xk, Uk, processNoises);
}

/**
 * 
 * If it fails to estimate motion:
 *      we use constant motion model to update the current state.
 */
bool Tracking::TrackReferenceKeyFrame()
{
    // std::cout << "Track reference keyframe with KF ID: " << mpCurrentKeyFrame->mnId << std::endl;
    // std::cout << "With pose: " << std::endl << mpCurrentKeyFrame->GetPose() << std::endl;
    bool bTrackOK = 0;
    std::vector<cv::Point2f> prev, curr;

    if (mConfigLoader.trackMode == 0)
    {

        std::size_t id = mCurrentFrame.mnId;
        cv::Mat img1 = mCurrentFrame.mCartImage;
        cv::Mat img2 = mpCurrentKeyFrame->mCartImage;
        std::vector<cv::KeyPoint> keypoints1 = mCurrentFrame.mvKeypoints;
        std::vector<cv::KeyPoint> keypoints2 = mpCurrentKeyFrame->mvKeypoints;
        cv::Mat descriptors1 = mCurrentFrame.mvDescriptors;
        cv::Mat descriptors2 = mpCurrentKeyFrame->mvDescriptors;

        // Compute matches
        std::vector<cv::DMatch> matches;
        ComputeMatches(id, img1, img2, keypoints1, keypoints2, descriptors1, descriptors2,
                       matches, mConfigLoader);
        mCurrentFrame.nMatches = matches.size();

        // We don't want this happen.
        if (mCurrentFrame.nMatches < 4)
        {
            return bTrackOK = 0;
        }

        for (int i = 0; i < matches.size(); i++)
        {
            //-- Get the keypoints from the matches
            curr.push_back(keypoints1[matches[i].queryIdx].pt);
            prev.push_back(keypoints2[matches[i].trainIdx].pt);
        }

        // Compute relative motion
        cv::Mat RT_SVD;
        bTrackOK = ComputeRelativePose(prev, curr, RT_SVD);

        if (bTrackOK)
        {
            // std::cout << "Matches between current frame and the keyframe: " << matches.size() << std::endl;
            std::vector<MapPoint *> mvpKFMapPoints = mpCurrentKeyFrame->GetMapPointMatches();

            // Assign keypoint correspondence to map point
            for (int i = 0; i < matches.size(); i++)
            {

                MapPoint *pMP = mvpKFMapPoints[matches[i].trainIdx];
                mCurrentFrame.mvpMapPoints[matches[i].queryIdx] = pMP;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->nObs++;
                // std::cout << "Current keypoint " << matches[i].queryIdx << " is matched wiht Keyframe keypoint " << matches[i].trainIdx <<std::endl;
            }

            cv::Mat Tcw_SVD = mpCurrentKeyFrame->GetPose() * RT_SVD;
            // Set RT of current frame respect to keyframe
            mTcwCurrent = Tcw_SVD.clone();
            mCurrentFrame.SetRT(RT_SVD);
            mCurrentFrame.TcwSVD = Tcw_SVD.clone();
        }

        // int index = 0;
        // for (std::vector<MapPoint *>::iterator vit = mCurrentFrame.mvpMapPoints.begin(), vend = mCurrentFrame.mvpMapPoints.end(); vit != vend; vit++)
        // {
        //     MapPoint *pMP = *vit;

        //     if (!pMP)
        //     {
        //         index++;
        //     }
        // }
    }
    else
    {
        // Tracking feature mode 1.
        std::size_t id = mCurrentFrame.mnId;
        mCurrentFrame.SetReferenceKeyFrame(mpCurrentKeyFrame);
        // std::cout << "mpCurrentKeyFrame mnId: " << mpCurrentKeyFrame->mnId << std::endl;
        cv::Mat img1 = mCurrentFrame.mCartImage;
        cv::Mat img2 = mpCurrentKeyFrame->mCartImage;
        std::vector<cv::Point2f> p1, p2;
        std::vector<cv::KeyPoint> keypoints1; // current keypoints vector
        std::vector<cv::KeyPoint> keypoints2 = mpCurrentKeyFrame->mvKeypoints;
        for (int i = 0; i < keypoints2.size(); i++)
        {
            p2.push_back(keypoints2[i].pt);
        }
        std::vector<int> matchesTrainId;
        std::vector<uchar> status;
        std::vector<float> err;

        // auto start = std::chrono::high_resolution_clock::now();
        cv::calcOpticalFlowPyrLK(img2, img1, p2, p1, status, err);
        // auto stop = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        // std::cout << "calcOpticalFlowPyrLK computation time is: " << duration.count() << " milliseconds." << std::endl;
        // std::cout << "p2.size: " << p2.size() << std::endl;
        // std::cout << "p1.size: " << p1.size() << std::endl;

        for (int i = 0; i < p2.size(); i++)
        {
            if (status[i] == 1)
            {
                curr.push_back(p1[i]);
                prev.push_back(p2[i]);
                matchesTrainId.push_back(i);
                keypoints1.push_back(cv::KeyPoint(p1[i], keypoints2[i].size));
            }
        }

        // Here the mnMatchesInliers depends on the tracked keypoint between keyframe and current frame
        mnMatchesInliers = curr.size();

        // Compute relative motion
        cv::Mat RT_SVD;
        bTrackOK = ComputeRelativePose(prev, curr, RT_SVD);
        mCurrentFrame.SetKeypoints(keypoints1); // set keypoints relate to the keyframe, this is used in local mapping BA.
        cv::Mat img_keypoints;
        cv::drawKeypoints(img1, keypoints1, img_keypoints);
        // resize it
        cv::resize(img_keypoints, img_keypoints, cv::Size(800, 800), cv::INTER_NEAREST);
        mCurrentFrame.mKeyPointsImage = img_keypoints;

        if (bTrackOK)
        {
            // std::cout << "Matches between current frame and the keyframe: " << curr.size() << std::endl;
            std::vector<MapPoint *> mvpKFMapPoints = mpCurrentKeyFrame->GetMapPointMatches();
            // Assign keypoint correspondence to map point
            for (int i = 0; i < prev.size(); i++)
            {

                MapPoint *pMP = mvpKFMapPoints[matchesTrainId[i]];
                mCurrentFrame.mvpMapPoints[i] = pMP;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->nObs++;
                // std::cout << "Current keypoint " << matches[i].queryIdx << " is matched wiht Keyframe keypoint " << matches[i].trainIdx <<std::endl;
            }

            cv::Mat Tcw_SVD = mpCurrentKeyFrame->GetPose() * RT_SVD;
            // Set RT of current frame respect to keyframe
            mTcwCurrent = Tcw_SVD.clone();
            mCurrentFrame.TcwSVD = Tcw_SVD.clone();
            mCurrentFrame.SetPose(mTcwCurrent); // For Feature tracker
        }

        // If tracked points number between current frame and keyframe less than threshold
        // Generate new keypoints but not Feature.
        if (mnMatchesInliers < mConfigLoader.minimumTrackPoints)
        {
            std::vector<cv::KeyPoint> keypointsToBeUpdated = mCurrentFrame.mvKeypoints;
            std::vector<cv::KeyPoint> keypointsCandidate;
            cv::Mat descriptors;

            // Extract new keypoints
            FeatureExtractionANMS(img1, mConfigLoader, keypointsCandidate, descriptors);

            // Todo: Make bucket grid. Sort bucket based on number of feature in a grid in descendent order.

            // Sort new keypoints based on their conerness
            std::vector<float> responseVector;
            for (unsigned int i = 0; i < keypointsCandidate.size(); i++)
                responseVector.push_back(keypointsCandidate[i].response);
            std::vector<int> Indx(responseVector.size());
            std::iota(std::begin(Indx), std::end(Indx), 0);
            cv::sortIdx(responseVector, Indx, CV_SORT_DESCENDING);
            std::vector<cv::KeyPoint> keyPointsSorted;
            for (unsigned int i = 0; i < keypointsCandidate.size(); i++)
                keyPointsSorted.push_back(keypointsCandidate[Indx[i]]);

            for (int i = 0; i < keyPointsSorted.size(); i++)
            {
                cv::KeyPoint keypt = keyPointsSorted[i];
                keypointsToBeUpdated.push_back(keypt);
                if (keypointsToBeUpdated.size() >= mConfigLoader.maximumTrackPoints)
                    break;
            }

            mCurrentFrame.SetKeypoints(keypointsToBeUpdated);
            // In the local mapping, new map points will be created with descriptor.
            // So we create an empty descriptor mat so that we can fool the local mapping ......
            cv::Mat vEmptyDescriptors = cv::Mat(keypointsToBeUpdated.size(), 1, CV_8UC1);
            mCurrentFrame.SetDescriptors(vEmptyDescriptors);
        }

        // std::cout << "Size of mvKeypoints in Current frame: " << mCurrentFrame.mvKeypoints.size() << std::endl;
    }

    mCurrentFrame.mnMatchesWithLastKeyFrame = mnMatchesInliers;


    return bTrackOK;
}

void Tracking::ComputeMatches(std::size_t index, cv::Mat img1, cv::Mat img2, const std::vector<cv::KeyPoint> keypoints1, const std::vector<cv::KeyPoint> keypoints2,
                              const cv::Mat descriptors1, const cv::Mat descriptors2, std::vector<cv::DMatch> &matches, const ConfigLoader mConfigLoader)
{

    // std::cout << "Enter ComputeMatches." << std::endl;
    // std::cout << "rows: " << descriptors1.rows << " and cols: " << descriptors1.cols << " of descriptors 1." << std::endl;
    // std::cout << "rows: " << descriptors2.rows << " and cols: " << descriptors2.cols << " of descriptors 2." << std::endl;
    // std::cout << "type: " << descriptors1.type() << " of descriptors 1." << std::endl;
    // std::cout << "type: " << descriptors2.type() << " of descriptors 2." << std::endl;

    // cv::namedWindow("Descriptors1", cv::WINDOW_AUTOSIZE); // Create a window for display.
    // //cv::imshow("descriptors", descriptors1.t());
    // cv::namedWindow("Descriptors2", cv::WINDOW_AUTOSIZE); // Create a window for display.
    // //cv::imshow("descriptors", descriptors2.t());
    // //cv::waitKey(0);

    // ----------------------------------------------------Ｋeypoints matching----------------------------------------------------
    std::vector<cv::Point2f> curr;
    std::vector<cv::Point2f> prev;

    ImageKeypointMatching(mConfigLoader, img1, keypoints1, keypoints2, descriptors1, descriptors2, curr, prev, matches);
    // std::string str = "Reference frame" ;
    // cv::putText(img2,
    //             str,
    //             cv::Point(img2.cols/2 -20, 20),               // Coordinates
    //             cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
    //             1.5,                            // Scale. 2.0 = 2x bigger
    //             cv::Scalar(255, 255, 255),      // BGR Color
    //             1);                             // Line Thickness (Optional)

    // -----------------------------------------------------Draw matches--------------------------------------------------------
    //-- Draw matches
    cv::Mat img_matches;
    cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches, cv::Scalar::all(-1),
                    cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    mCurrentFrame.mImageMatches = img_matches.clone();
    // // cv::namedWindow("Keypoints matching", cv::WINDOW_AUTOSIZE); // Create a window for display.
    // //-- Show detected matches
    // cv::resize(img_matches, img_matches, cv::Size(1200, 600), cv::INTER_NEAREST);

    // std::string str_tracking_ok = "Tracking ok: " + bool_cast(bTrackOK);
    // cv::putText(img_matches,
    //             str_tracking_ok,
    //             cv::Point(5, 20),               // Coordinates
    //             cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
    //             1.0,                            // Scale. 2.0 = 2x bigger
    //             cv::Scalar(255, 255, 255),      // BGR Color
    //             1);                             // Line Thickness (Optional)

    //cv::imshow("Good Matches", img_matches);
    //cv::waitKey(0);

    // Upadate mnMatchesInliers
    mnMatchesInliers = matches.size();

    // std::cout << "Exit ComputeMatches." << std::endl;
}

bool Tracking::ComputeRelativePose(const std::vector<cv::Point2f> prev, const std::vector<cv::Point2f> curr, cv::Mat &RT)
{
    bool bTrackOK = 0;
    // ----------------------------------------------------Motion estimation----------------------------------------------------
    // Using OpenCV motion solver
    // cv::videostab::MotionEstimatorRansacL2 motionEstimator(cv::videostab::MM_RIGID);
    // cv::Mat RT = motionEstimator.estimate(prev, curr, &bTrackOK);

    // Using SVD to estimate motion
    RT = estimateMotionSVD(mConfigLoader, prev, curr, bTrackOK);
    // // Using ICP to estimate motion
    // RT = estimateMotionICP(mConfigLoader, prev, curr, bTrackOK);
    // std::cout << "ICP RT:\n"
    //           << RT << std::endl;

    // Using backward projection to estimate motion, using g2o

    RT.convertTo(RT, CV_32F);

    return bTrackOK;
}

/**
 * reference: https://en.wikipedia.org/wiki/Rotation_matrix
 * tan(a) = sin(a) / cos(a); a = arctan(tan(a))
 * Rotation matrix = [cos(a) -sin(a);
 *                   sin(a)  cos(a)]]
*/
bool Tracking::NeedNewKeyFrame()
{

    cv::Mat Tcw_keyframe = mpCurrentKeyFrame->GetPose();
    cv::Mat Tcw_current = mCurrentFrame.GetPose();

    float distance_diff, angle_diff;
    float sin_key = Tcw_keyframe.at<float>(1, 0);
    float cos_key = Tcw_keyframe.at<float>(0, 0);
    float sin_curr = Tcw_current.at<float>(1, 0);
    float cos_curr = Tcw_current.at<float>(0, 0);

    float angle_key = atan2(sin_key, cos_key);
    float angle_curr = atan2(sin_curr, cos_curr);

    distance_diff = sqrt(pow((Tcw_current.at<float>(0, 2) - Tcw_keyframe.at<float>(0, 2)), 2) +
                         pow((Tcw_current.at<float>(1, 2) - Tcw_keyframe.at<float>(1, 2)), 2));

    angle_diff = fabs(angle_curr - angle_key);

    // Condition 1a: translation to the last keyframe
    const bool c1a = distance_diff >= mConfigLoader.keyFrameDistance;
    // Condition 1b: angle difference to the last keyframe
    const bool c1b = angle_diff >= mConfigLoader.keyFrameAngle;
    // Condition 1c: few tracked points compared to reference keyframe
    const bool c1c = mnMatchesInliers < 10; // Hard coded for now

    // std::cout << "keyframe id: " << mpCurrentKeyFrame->mnFrameId << std::endl;
    // std::cout << "angle_diff: " << angle_diff <<  std::endl;
    // std::cout << "distance_diff: " << distance_diff <<  std::endl;
    // std::cout << "mnMatchesInliers: " << mnMatchesInliers <<  std::endl;

    if (c1a || c1b || c1c)
    {
        // std::cout << "Need new keyframe." << std::endl;
        return true;
    }
    else
    {
        return false;
    }
}

// bool Tracking::LocaliseLocalMap()
// {
//     return false;
// }

void Tracking::ClearLocalWindow()
{
    while (mvWindowFrames.size() > 1)
    {
        mvWindowFrames.erase(mvWindowFrames.begin());
    }
}

// Associate with local map points and keyframes,
// compute the current pose optimized with map points position using optimization
bool Tracking::TrackLocalMap()
{
    if (mConfigLoader.trackMode == 0)
    {
        /* Use feature matcher to compute poses */


        // We have an estimation of the camera pose and some map points tracked in the frame.
        // We retrieve the local map and try to find matches to points in the local map.
        // This is 3D to 2D matching
        UpdateLocalMap();
        SearchLocalPoints();

        // Optimize Pose respect to the local map
        mnMatchesInliers = Optimizer::PoseOptimization(&mCurrentFrame, mConfigLoader);
        // Retreive inlier matches respect the map
        return mnMatchesInliers >= mConfigLoader.minMatchesBetweenFrames;
    }
    else
    {
        /* Use feature tracker to compute poses */

        // Create a new thread to compute M2DP
        std::thread new_thread(&Frame::ComputeGlobalDescriptor, &mCurrentFrame);

        // Retreive the local frames pointer
        std::vector<Frame *> vpWindowFrames = GetLocalWindowFrames();

        // Track new frame in the local window
        cv::Mat RT;
        mpFeatureTracker->TrackNewFrame(&mCurrentFrame, &mPreviousFrame, vpWindowFrames, mvRT, RT, mConfigLoader);

        // depends on the number of matches between current frame and keyframe
        mnMatchesInliers = mCurrentFrame.mvKeypoints.size();

        // Slide the window
        if (mvWindowFrames.size() == mConfigLoader.maximumTrackFrames)
        {
            mvWindowFrames.erase(mvWindowFrames.begin());
            mvRT.erase(mvRT.begin());
        }

        // Insert current frame into local window only when the motion is large
        const float distance_threshold = 0.1;
        const float angle_threshold = 0.01;
        float travelDistance = sqrt(RT.at<float>(0, 2) * RT.at<float>(0, 2) + RT.at<float>(1, 2) * RT.at<float>(1, 2));
        float angle_yaw = abs(atan2(RT.at<float>(1, 0), RT.at<float>(0, 0)));
        if (travelDistance >= distance_threshold || angle_yaw >= angle_threshold)
        {
            mvWindowFrames.push_back(mCurrentFrame);
            mvRT.push_back(RT);
        }


        new_thread.join();
        // std::cout << mCurrentFrame.mvPointCloud.rows << std::endl;
        return true;
    }

    // return false;
}

// Update the local keyframe and mappoint for viewer
void Tracking::UpdateLocalMap()
{
    // std::cout << "Tracker::UpdateLocalMap()\n";
    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();

    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
}

void Tracking::ConstantMotionModel()
{
    //Rotation matrix = [cos(a) -sin(a);
    //                   sin(a)  cos(a)]]

    // printf("Enter ConstantMotionModel\n");
    // printf("Press Enter to continue...\n");
    // std::getchar();

    double time_difference = mCurrentTimestamp - mPreviousTimestamp;
    double translationX = mVelocityX * time_difference;
    double translationY = mVelocityY * time_difference;
    double translationYaw = mVelocityYaw * time_difference;

    cv::Mat Tcw = cv::Mat::eye(3, 3, CV_32F);

    Tcw.at<float>(0, 2) = translationX + Tcw_prev.at<float>(0, 2);
    Tcw.at<float>(1, 2) = translationY + Tcw_prev.at<float>(1, 2);

    double prev_yaw = atan2(Tcw_prev.at<float>(1, 0), Tcw_prev.at<float>(0, 0));
    double curr_yaw = prev_yaw + translationYaw;
    Tcw.at<float>(0, 0) = cos(curr_yaw);
    Tcw.at<float>(0, 1) = -sin(curr_yaw);
    Tcw.at<float>(1, 0) = sin(curr_yaw);
    Tcw.at<float>(1, 1) = cos(curr_yaw);
    Tcw.convertTo(Tcw, CV_32F);
    mCurrentFrame.SetPose(Tcw);
}

void Tracking::ComputeVelocity()
{

    cv::Mat Tcw = mCurrentFrame.GetPose();

    mVelocityX = (Tcw.at<float>(0, 2) - Tcw_prev.at<float>(0, 2)) / (mCurrentTimestamp - mPreviousTimestamp);
    mVelocityY = (Tcw.at<float>(1, 2) - Tcw_prev.at<float>(1, 2)) / (mCurrentTimestamp - mPreviousTimestamp);

    double prev_yaw = atan2(Tcw_prev.at<float>(1, 0), Tcw_prev.at<float>(0, 0));
    double curr_yaw = atan2(Tcw.at<float>(1, 0), Tcw.at<float>(0, 0));
    mVelocityYaw = (curr_yaw - prev_yaw) / (mCurrentTimestamp - mPreviousTimestamp);

    // Update variables
    mPreviousTimestamp = mCurrentTimestamp;
    Tcw_prev = Tcw;
}

void Tracking::UpdateLocalKeyFrames()
{

    // Each map point vote for the keyframes in which it has been observed
    std::map<KeyFrame *, int> keyframeCounter;
    for (int i = 0; i < mCurrentFrame.N; i++)
    {
        if (mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
            if (!pMP)
                continue;
            if (!pMP->isBad())
            {
                // std::cout << "UpdateLocalKeyFrames::ith mappoint: " <<  i << " is good." << std::endl;

                const std::map<KeyFrame *, size_t> observations = pMP->GetObservations();
                for (std::map<KeyFrame *, size_t>::const_iterator it = observations.begin(), itend = observations.end(); it != itend; it++)
                {
                    keyframeCounter[it->first]++;
                    // std::cout <<"Observed by keyframe: " << (it->first)->mnId << std::endl;
                }
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i] = NULL;
            }
        }
    }

    if (keyframeCounter.empty())
        return;

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

    // Preserve keyframes that have more than n points that currently observed
    int nPoints = 5;
    for (std::map<KeyFrame *, int>::const_iterator it = keyframeCounter.begin(), itEnd = keyframeCounter.end(); it != itEnd; it++)
    {
        KeyFrame *pKF = it->first;

        if (pKF->isBad())
            continue;

        // if a keyframe has more than n map point being observed by current frame, use it.
        if (it->second > nPoints)
        {
            mvpLocalKeyFrames.push_back(it->first);
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
        }
    }
    // std::cout << "UpdateLocalKeyFrames::Number of mvpLocalKeyFrames: " << mvpLocalKeyFrames.size() << std::endl;
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for (int i = 0; i < mCurrentFrame.mvpMapPoints.size(); i++)
    {
        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
        if (pMP)
        {
            mvpLocalMapPoints.push_back(pMP);
        }
    }

    // // Search through all the connected keyframe for the map points
    // int nMapPoints = 0;
    // for(std::vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    // {
    //     KeyFrame* pKF = *itKF;
    //     const std::vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

    //     for(std::vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
    //     {
    //         MapPoint* pMP = *itMP;
    //         nMapPoints++;

    //         if(!pMP)
    //             continue;
    //         if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
    //             continue;
    //         if(!pMP->isBad())
    //         {
    //             mvpLocalMapPoints.push_back(pMP);
    //             pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
    //         }
    //     }
    // }
    // std::cout <<"UpdateLocalPoints::Number of MapPoint added is: " << nMapPoints << std::endl;
}

/** 
 * 3D-2D matching
 * Perform matching between map points and the rest keypoints which are not associated yet
 */
void Tracking::SearchLocalPoints()
{

    int nToMatch = 0;

    // Project the local map point into the current frame to check visibility
    for (std::vector<MapPoint *>::iterator vit = mvpLocalMapPoints.begin(), vend = mvpLocalMapPoints.end(); vit != vend; vit++)
    {
        MapPoint *pMP = *vit;
        if (pMP->isBad())
        {
            *vit = static_cast<MapPoint *>(NULL);
        }
        else
        {
            if (mCurrentFrame.IsVisibleInRadius(pMP, mConfigLoader.cartImageScale, 5.0))
            {
                nToMatch++;
            }
            else
            {
                pMP->mbTrackInView = false; // Set to false so we are not going to perform matching on it
            }
        }
    }

    // Perform matching between map points and the rest keypoints
    if (nToMatch > 0)
    {
        Matcher *pMatcher = new Matcher(mConfigLoader);
        float radius_meters = 5.0;
        pMatcher->SearchByProjection(mCurrentFrame, mvpLocalMapPoints, radius_meters);
        delete pMatcher;
    }

    // // Project MapPoint into local frame
    // cv::Mat img_matches(mConfigLoader.cartImageScale*2 , mConfigLoader.cartImageScale*2, CV_8UC3, cv::Scalar(0));
    // for(int i=0; i< mCurrentFrame.mvpMapPoints.size(); i++){
    //     MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
    //     if(!pMP){
    //         // cv::circle(img_matches, cv::Point2f(, ), cvRound((double)8 / 2), cv::Scalar(0, 0, 255) ;
    //     }
    // }

    mnToMatchLocalMap = nToMatch;
}

void Tracking::CreateNewKeyFrame()
{
    while(!mCurrentFrame.IsM2dpReady()){
        usleep(1000);
    }

    KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap);

    pKF->ChangeParent(mpCurrentKeyFrame);
    mpLocalMapper->InsertKeyFrame(pKF);
    mpCurrentKeyFrame = pKF;

    if (mConfigLoader.trackMode == 1){
        // Compute RT respect to reference keyframe for all window frames
        for(int i=0; i< mvWindowFrames.size(); i++)
        {
            Frame* pFrame = &mvWindowFrames[i];
            cv::Mat RT2Keyframe = pKF->GetPoseInverse() * pFrame->GetPose();
            RT2Keyframe.convertTo(RT2Keyframe, CV_32F);
            pFrame->SetRT(RT2Keyframe); 
            // std::cout  << "Frame ID: " << pFrame->mnId << std::endl;
        }        
    }
}

int Tracking::GetCurrentFrameInliers()
{
    return mnMatchesInliers;
}

void Tracking::StoreNewPose()
{
    mvHistoryPoses.push_back(mCurrentFrame.GetPose());
}

// ToDo
bool Tracking::Relocalization()
{
    return false;
}
// } // namespace ROAM
