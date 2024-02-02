#include "LoopClosing.h"
#include "Converter.h"
#include "Optimizer.h"

LoopClosing::LoopClosing(Map *pMap, const ConfigLoader configLoader) : mbFinished(true),
                                                                       mpMap(pMap), mbFinishedGBA(true), mConfigLoader(configLoader), mLastLoopKFid(0), mnLoopCandidate(0)
{
    // Save LOOP RELATIVE POSE
    ofsLoop.open("/home/hong/Desktop/PoseError/LoopRelativePose.csv");
    ofsLoop << "LoopFrameID,CurrentFrameID,t_x,t_y,t_yaw" << std::endl;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker = pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper = pLocalMapper;
}

void LoopClosing::SetViewer(Viewer *pViewer)
{
    mpViewer = pViewer;
}

void LoopClosing::Run()
{
    mbFinished =false;
    // while(1)
    // {

    // std::cout << "\033[1;31m LoopClosing running \033[0m\n";

    // Check if there are keyframes in the queue
    if (CheckNewKeyFrames())
    {
        // Detect loop candidates
        if (DetectLoop())
        {

            // Compute transformation
            if (ComputeSE2())
            {
                // auto start = std::chrono::high_resolution_clock::now();

                mpViewer->PlotLoop(mpCurrentKF, mpMatchedKF);
                // auto stop = std::chrono::high_resolution_clock::now();
                // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
                // std::cout << "0 computation time is: " << duration.count() << " milliseconds." << std::endl;

                // Perform loop fusion and pose graph optimization
                CorrectLoop();
                // auto stop1 = std::chrono::high_resolution_clock::now();
                // auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start);
                // std::cout << "1 computation time is: " << duration1.count() << " milliseconds." << std::endl;
                mpViewer->CorrectLoop();
                // auto stop2 = std::chrono::high_resolution_clock::now();
                // auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(stop2 - start);
                // std::cout << "2 computation time is: " << duration2.count() << " milliseconds." << std::endl;
                // std::getchar();

            }
            else
            {
                std::cout << "Failed to compute SE2 in the loop closer." << std::endl;
                // std::getchar();
            }


        }
    }

    //     usleep(5000);
    // }

    SetFinish();
}

// Loop closing detection
bool LoopClosing::DetectLoop()
{
    // auto start = std::chrono::high_resolution_clock::now();

    {
        std::unique_lock<std::mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front(); // Remove it from the list
    }

    // Use PCA to see if point cloud is dominated by one component, if so, reject it.
    const double pcaThreshold = 3.0;
    cv::Mat pointCloudForward = mpCurrentKF->mvPointCloud.clone();
    cv::PCA pca_analysis(pointCloudForward, cv::Mat(), cv::PCA::DATA_AS_ROW);
    if (pca_analysis.eigenvalues.at<double>(0) / pca_analysis.eigenvalues.at<double>(1) > pcaThreshold)
    {
        return false;
    }

    // //If the map contains less than N KF or less than N KF have passed from last loop detection
    // const int N = 10;
    // if (mpCurrentKF->mnId < mLastLoopKFid + N )
    // {
    //     return false;
    // }

    const double loopSimilarityThres = 0.2;
    mbReverseLoop = false;

    // Compare descriptor distance
    cv::Mat currentFeature = mpCurrentKF->GetFeature();
    std::vector<KeyFrame *> potentialKFs = SamplePotentialLoops();
    // std::cout << "LoopClosing:: number of potential loops: " << potentialKFs.size() << std::endl;
    double minDist = 1.0;
    double minDistReverse = 1.0;
    int minIndex = -1;
    int minIndexReverse = -1;
    for (std::vector<KeyFrame *>::iterator it = potentialKFs.begin(); it != potentialKFs.end(); ++it)
    {
        KeyFrame *pKF = *it;
        cv::Mat feature = pKF->GetFeature();
        // cv::Mat featureReverse = pKF->GetFeatureReverse();

        double distanceForward = cv::norm(currentFeature, feature, cv::NORM_L2);
  
        // double distanceReverse = cv::norm(currentFeature, featureReverse, cv::NORM_L2);
        if (distanceForward < minDist)
        {
            minDist = distanceForward;
            minIndex = pKF->mnFrameId;
        }
        // if (distanceReverse < minDistReverse)
        // {
        //     minDistReverse = distanceReverse;
        //     minIndexReverse = pKF->mnFrameId;
        // }

        // Forward loop
        if (distanceForward < loopSimilarityThres)
        {
            mpMatchedKF = pKF;
            mLastLoopKFid = mpCurrentKF->mnId; // update the last loop KFid to avoid too many loops detected afterwards

            // Check distance, if large then we accept it. If too small, means we have a loop nearby no need to correct for now.
            cv::Mat Tcw_matched = mpMatchedKF->GetPose();
            cv::Mat Tcw_current = mpCurrentKF->GetPose();
            double distBeforeCorrection = sqrt(pow((Tcw_current.at<float>(0, 2) - Tcw_matched.at<float>(0, 2)), 2) +
                                               pow((Tcw_current.at<float>(1, 2) - Tcw_matched.at<float>(1, 2)), 2));

            if (distBeforeCorrection < 5.0) // if distance small, means either we just had a loop or we are tracking good
            {
                return false;
            }

            std::cout << "\033[1;32mLoop detected!\033[0m" << std::endl;
            std::cout << "Between frame: " << mpCurrentKF->mnFrameId << " and frame: " << mpMatchedKF->mnFrameId << "\033[1;37m \033[0m" << std::endl;
            // std::cout << "Feature distance: " << distanceForward << std::endl;
            // auto stop = std::chrono::high_resolution_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            // std::cout << "DetectLoop computation time is: " << duration.count() << " milliseconds." << std::endl;
            return true;
        }

        // // Reverse loop
        // if (distanceReverse < loopSimilarityThres)
        // {
        //     mpMatchedKF = pKF;
        //     mLastLoopKFid = mpCurrentKF->mnId; // update the last loop KFid to avoid too many loops detected afterwards

        //     // Check distance, if large then we accept it. If too small, means we have a loop nearby no need to correct for now.
        //     cv::Mat Tcw_matched = mpMatchedKF->GetPose();
        //     cv::Mat Tcw_current = mpCurrentKF->GetPose();
        //     double distBeforeCorrection = sqrt(pow((Tcw_current.at<float>(0, 2) - Tcw_matched.at<float>(0, 2)), 2) +
        //                                        pow((Tcw_current.at<float>(1, 2) - Tcw_matched.at<float>(1, 2)), 2));

        //     if (distBeforeCorrection < 5.0) // if distance small, means either we just had a loop or we are tracking good
        //     {
        //         return false;
        //     }

        //     mbReverseLoop = true;

        //     std::cout << "\033[1;32mReverse loop detected!\033[0m" << std::endl;
        //     std::cout << "Between frame: " << mpCurrentKF->mnFrameId << " and frame: " << mpMatchedKF->mnFrameId << "\033[1;37m \033[0m" << std::endl;

        //     auto stop = std::chrono::high_resolution_clock::now();
        //     auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        //     std::cout << "DetectLoop computation time is: " << duration.count() << " milliseconds." << std::endl;
        //     std::getchar();

        //     return true;
        // }
    }
    // std::cout << "Potential frame: " << minIndex << " has minimum distance: " << minDist << std::endl;
    // std::cout << "Potential frame: " << minIndexReverse << " has minimum distance: " << minDistReverse << std::endl;

    return false;
}

// ToDo: use covariance
std::vector<KeyFrame *> LoopClosing::SamplePotentialLoops()
{
    std::vector<KeyFrame *> potentialLoops;
    cv::Mat currentPose = mpCurrentKF->GetPose();

    std::vector<KeyFrame *> vKFs = mpMap->GetAllKeyFrames();
    for (std::vector<KeyFrame *>::iterator it = vKFs.begin(); it != vKFs.end(); ++it)
    {
        KeyFrame *pKF = *it;

        // Avoid nearby self-matching
        if (abs(mpCurrentKF->mnFrameId - pKF->mnFrameId) > 100)
        {
            cv::Mat pose = pKF->GetPose();

            double dist = sqrt(pow(currentPose.at<float>(0, 2) - pose.at<float>(0, 2), 2) +
                               pow(currentPose.at<float>(1, 2) - pose.at<float>(1, 2), 2));

            // Check whether the frame is within the pose covariance or hard-coded distance in meters
            if (dist < 500)

                potentialLoops.push_back(pKF);
        }
    }

    return potentialLoops;
}

bool LoopClosing::ComputeSE2()
{
    // std::cout << "Compute SE2." << std::endl;

    std::size_t idCurrentKF = mpCurrentKF->mnFrameId;
    std::size_t idMatchedKF = mpMatchedKF->mnFrameId;
    std::string frame_id = std::to_string(idCurrentKF);
    std::string match_id = std::to_string(idMatchedKF);

    cv::Mat img1 = mpCurrentKF->mCartImage;
    cv::Mat img2 = mpMatchedKF->mCartImage;
    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors1;
    cv::Mat descriptors2;
    std::vector<cv::Point2f> curr;
    std::vector<cv::Point2f> prev;
    bool bOK;
    cv::Mat RT = cv::Mat::eye(3, 3, CV_32F);

    if (mConfigLoader.trackMode == 1)
    {

        cv::Mat currMat = mpCurrentKF->mvPointCloud.clone();
        cv::Mat prevMat = mpMatchedKF->mvPointCloud.clone();

        /* Using Point Cloud */

        // Compute relative motion using ICP
        // RT = estimatePointCloudMotionICP(mConfigLoader, prevMat, currMat, bOK);

        /* Using Image */
        double rotated_angle_degree;
        if (mbReverseLoop)
        {
            rotated_angle_degree = 180;
        }
        else
        {

            //Perform PCA analysis
            cv::PCA pca_prev(prevMat, cv::Mat(), cv::PCA::DATA_AS_ROW);
            cv::PCA pca_curr(currMat, cv::Mat(), cv::PCA::DATA_AS_ROW);
            cv::Point2d first_vector_prev = cv::Point2d(pca_prev.eigenvectors.at<double>(0, 0), pca_prev.eigenvectors.at<double>(0, 1));
            cv::Point2d first_vector_curr = cv::Point2d(pca_curr.eigenvectors.at<double>(0, 0), pca_curr.eigenvectors.at<double>(0, 1));
            double angle_vec_prev = atan2(first_vector_prev.y, first_vector_prev.x);
            double angle_vec_curr = atan2(first_vector_curr.y, first_vector_curr.x);
            double angle_diff = angle_vec_prev - angle_vec_curr;
            rotated_angle_degree = angle_diff * 180 / M_PI;
        }

        if (fabs(rotated_angle_degree) > 8)
        {
            return false;
        }

        // Rotate the current keyframe image so that we move the features close and we can "track" them.
        cv::Mat rotated_img1;
        cv::Point2f pc(img1.cols / 2., img1.rows / 2.);
        cv::Mat affineTransform = cv::getRotationMatrix2D(pc, rotated_angle_degree, 1.0);
        cv::warpAffine(img1, rotated_img1, affineTransform, img1.size());
        // std::cout << "affineTransform: " << affineTransform << std::endl;

        /*----------------------------------------------- Using descriptor -----------------------------------------------------*/
        // FeatureExtractionANMS(rotated_img1, mConfigLoader, keypoints1, descriptors1);
        // FeatureExtractionANMS(img2, mConfigLoader, keypoints2, descriptors2);

        // // Match keypoints
        // std::vector<cv::DMatch> matches;
        // ImageKeypointMatching(mConfigLoader, img1, keypoints1, keypoints2, descriptors1, descriptors2, curr, prev, matches);
        // curr.clear();
        // prev.clear();
        // for (int i = 0; i < matches.size(); i++)
        // {
        //     //-- Get the keypoints from the matches
        //     curr.push_back(keypoints1[matches[i].queryIdx].pt);
        //     prev.push_back(keypoints2[matches[i].trainIdx].pt);
        // }

        /*--------------------------------------------------- Using trackers ---------------------------------------------------*/
        std::vector<cv::Point2f> p1, p2;
        std::vector<cv::DMatch> matches;
        std::vector<uchar> status;
        std::vector<float> err;
        keypoints2 = mpMatchedKF->mvKeypoints;
        for (int i = 0; i < keypoints2.size(); i++)
        {
            p2.push_back(keypoints2[i].pt);
        }

        cv::calcOpticalFlowPyrLK(img2, rotated_img1, p2, p1, status, err);
        for (int i = 0; i < p2.size(); i++)
        {
            if (status[i] == 1)
            {
                matches.push_back(cv::DMatch(i, i, 0.0));
            }
        }

        // Remove moving targets
        DynamicTargetRemoval(mConfigLoader, matches, p2, p1);

        curr.clear();
        prev.clear();
        keypoints2.clear();
        std::vector<cv::DMatch> matches4vis;
        double c = cos(-rotated_angle_degree / 180 * M_PI);
        double s = sin(-rotated_angle_degree / 180 * M_PI);

        for (int i = 0; i < matches.size(); i++)
        {
            prev.push_back(p2[matches[i].trainIdx]);
            keypoints2.push_back(cv::KeyPoint(p2[matches[i].trainIdx], 0.0));

            // Inverse rotation
            cv::Point2f pt1Rotated = p1[matches[i].queryIdx];
            cv::Point2f pt1Origin;
            float localX = pt1Rotated.x - pc.x;
            float localY = pc.y - pt1Rotated.y;
            float NewX = localX * c - localY * s;
            float NewY = localX * s + localY * c;
            float pixelX = NewX + pc.x;
            float pixelY = pc.y - NewY;
            pt1Origin.x = pixelX;
            pt1Origin.y = pixelY;
            // ptOrigin.x = (ptRotated.x - pc.x) * c - (ptRotated.y - pc.y) * s + pc.x;
            // ptOrigin.y = (ptRotated.x - pc.x) * s + (ptRotated.y - pc.y) * c + pc.y;
            curr.push_back(pt1Origin);
            keypoints1.push_back(cv::KeyPoint(pt1Origin, 0.0));

            matches4vis.push_back(cv::DMatch(i, i, 0.0));
        }

        // Set numnber of matches between loop
        mpCurrentKF->SetNumberMatchesWithLoop(matches.size());

        // -----------------------------------------------------Draw matches--------------------------------------------------------
        //-- Draw matches
        // cv::Mat img_matches;
        // cv::drawMatches(img1, keypoints1, img2, keypoints2, matches4vis, img_matches, cv::Scalar::all(-1),
        //                 cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        // cv::Mat dst;
        // cv::hconcat(img1, img2, dst);
        // std::string filename = "/home/hong/Desktop/PoseError/LoopImages/" +
        //                        std::string(6 - frame_id.length(), '0').append(frame_id) + "_" +
        //                        std::string(6 - match_id.length(), '0').append(match_id) + ".jpg";
        // // cv::imwrite(filename, dst);
        // cv::imwrite(filename, img_matches);

        // //-- Show detected matches
        // cv::namedWindow("Keypoints matching", cv::WINDOW_AUTOSIZE); // Create a window for display.
        // cv::resize(img_matches, img_matches, cv::Size(1200, 600), cv::INTER_NEAREST);
        // cv::imshow("Loop Matches", img_matches);
        // cv::waitKey(0);

        double factorT0 = 0.5; // The factor of T(t=0)
        Eigen::Vector3d velocityPrev = mpMatchedKF->mVelocity;
        for (int i = 0; i < prev.size(); i++)
        {
            cv::Point2d image_point = prev[i];
            cv::Point2f obs_xy = imageFrame2LocalFrame(mConfigLoader.cartImageHeight, mConfigLoader.cartImageWidth,
                                                       mConfigLoader.cartImageScale, image_point);

            double theta = atan2(obs_xy.y, obs_xy.x);
            double factorT;
            if (theta > 0)
            {
                factorT = ((M_PI * 2 - theta)) / (2 * M_PI);
            }
            else
            {
                factorT = (-theta) / (2 * M_PI);
            }
            double frameTimeDiff = 0.25;
            double deltaT = frameTimeDiff * (factorT - factorT0);
            g2o::SE2 T0t(velocityPrev[0] * deltaT, velocityPrev[1] * deltaT, velocityPrev[2] * deltaT);
            Eigen::Vector2d measurement(obs_xy.x, obs_xy.y);
            g2o::Vector2 vMeasCorrected = T0t * measurement;
            cv::Point2d ptCorrected = cv::Point2d(vMeasCorrected(0), vMeasCorrected(1));
            cv::Point2d ptCorrectedImage = localFrame2ImageFrame(mConfigLoader.cartImageHeight, mConfigLoader.cartImageWidth,
                                                                 mConfigLoader.cartImageScale, ptCorrected);
            prev[i] = ptCorrectedImage;
        }

        Eigen::Vector3d velocityCurr = mpCurrentKF->mVelocity;
        for (int i = 0; i < curr.size(); i++)
        {
            cv::Point2d image_point = curr[i];
            cv::Point2f obs_xy = imageFrame2LocalFrame(mConfigLoader.cartImageHeight, mConfigLoader.cartImageWidth,
                                                       mConfigLoader.cartImageScale, image_point);

            double theta = atan2(obs_xy.y, obs_xy.x);
            double factorT;
            if (theta > 0)
            {
                factorT = ((M_PI * 2 - theta)) / (2 * M_PI);
            }
            else
            {
                factorT = (-theta) / (2 * M_PI);
            }
            double frameTimeDiff = 0.25;
            double deltaT = frameTimeDiff * (factorT - factorT0);
            g2o::SE2 T0t(velocityCurr[0] * deltaT, velocityCurr[1] * deltaT, velocityCurr[2] * deltaT);
            Eigen::Vector2d measurement(obs_xy.x, obs_xy.y);
            g2o::Vector2 vMeasCorrected = T0t * measurement;
            cv::Point2d ptCorrected = cv::Point2d(vMeasCorrected(0), vMeasCorrected(1));
            cv::Point2d ptCorrectedImage = localFrame2ImageFrame(mConfigLoader.cartImageHeight, mConfigLoader.cartImageWidth,
                                                                 mConfigLoader.cartImageScale, ptCorrected);
            curr[i] =  ptCorrectedImage;                                                 
        }


        // Compute relative motion
        // RT = estimateMotionSVD(mConfigLoader, prev, curr, bOK);
        RT = estimateMotionICP(mConfigLoader, prev, curr, bOK);
        // cv::Mat preAlignT = cv::Mat::eye(3, 3, CV_32F);
        // preAlignT.at<float>(0, 0) = cos(rotated_angle_degree / 180 * M_PI);
        // preAlignT.at<float>(0, 1) = -sin(rotated_angle_degree / 180 * M_PI);
        // preAlignT.at<float>(1, 0) = sin(rotated_angle_degree / 180 * M_PI);
        // preAlignT.at<float>(1, 1) = cos(rotated_angle_degree / 180 * M_PI);

        // RT = preAlignT * RT;

        if (!bOK)
        {
            std::cout << "Fail to compute SE2." << std::endl;
            return false;
        }
    }
    else
    {
        /* Old version for computing SE2 */
        keypoints1 = mpCurrentKF->mvKeypoints;
        keypoints2 = mpMatchedKF->mvKeypoints;
        descriptors1 = mpCurrentKF->mvDescriptors;
        descriptors2 = mpMatchedKF->mvDescriptors;

        // Compute matches
        std::vector<cv::DMatch> matches;
        ImageKeypointMatching(mConfigLoader, img1, keypoints1, keypoints2, descriptors1, descriptors2, curr, prev, matches);

        curr.clear();
        prev.clear();
        for (int i = 0; i < matches.size(); i++)
        {
            //-- Get the keypoints from the matches
            curr.push_back(keypoints1[matches[i].queryIdx].pt);
            prev.push_back(keypoints2[matches[i].trainIdx].pt);
        }

        // -----------------------------------------------------Draw matches--------------------------------------------------------
        //-- Draw matches
        cv::Mat img_matches;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches, cv::Scalar::all(-1),
                        cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        cv::namedWindow("Keypoints matching", cv::WINDOW_AUTOSIZE); // Create a window for display.
        //-- Show detected matches
        cv::resize(img_matches, img_matches, cv::Size(1200, 600), cv::INTER_NEAREST);
        cv::imshow("Loop Matches", img_matches);
        cv::waitKey(0);

        // Compute relative motion
        // RT = estimateMotionSVD(mConfigLoader, prev, curr, bOK);
        RT = estimateMotionICP(mConfigLoader, prev, curr, bOK);
    }

    RT.convertTo(RT, CV_32F);
    mLoopRT = RT.clone();
    cv::Mat TcwUpdated = mpMatchedKF->GetPose() * RT;
    mTcm = TcwUpdated.clone();

    // std::cout << "RT is: " << std::endl
    //           << RT << std::endl;

    // Log relative pose
    ofsLoop << mpMatchedKF->mnFrameId << "," << mpCurrentKF->mnFrameId << "," << RT.at<float>(0, 2) << "," << RT.at<float>(1, 2) << ","
            << atan2(RT.at<float>(1, 0), RT.at<float>(0, 0)) << std::endl;

    // std::cout << "Relative angle in yaw: " << atan2(RT.at<float>(1, 0), RT.at<float>(0, 0)) * 180 / 3.141592
    //           << " in degrees." << std::endl;
    // std::cout << "Matched Tcw: " << std::endl
    //           << mpMatchedKF->GetPose() << std::endl;
    // std::cout << "Current Tcw before correction: " << std::endl
    //           << mpCurrentKF->GetPose() << std::endl;
    // std::cout << "Current Tcw after correction: " << std::endl
    //           << mTcm << std::endl;
    // std::getchargetchar();

    return bOK;
    // return true;
}

/** Perform essential graph optimization and correct 
 * the position of all the map points based on their first observation. 
*/
void LoopClosing::CorrectLoop()
{

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // // Wait until Local Mapping has effectively stopped
    // while(!mpLocalMapper->isStopped())
    // {
    //     usleep(1000);
    // }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected SE2 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    std::map<KeyFrame *, cv::Mat> CorrectedSE2, NonCorrectedSE2;
    cv::Mat Tjw = mTcm;
    CorrectedSE2[mpCurrentKF] = mTcm;
    cv::Mat Twj = mpCurrentKF->GetPoseInverse();

    {
        // Get Map Mutex
        std::unique_lock<std::mutex> lock(mpMap->mMutexMapUpdate);

        // Correct connected keyframes poses
        for (std::vector<KeyFrame *>::iterator vit = mvpCurrentConnectedKFs.begin(), vend = mvpCurrentConnectedKFs.end(); vit != vend; vit++)
        {
            KeyFrame *pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            cv::Mat Tji = Twj * Tiw;

            if (pKFi != mpCurrentKF)
            {
                cv::Mat CorrectedTiw = Tjw * Tji;
                //Pose corrected with the SE2 of the loop closure
                CorrectedSE2[pKFi] = CorrectedTiw;
            }

            // //Pose without correction
            // NonCorrectedSE2[pKFi]=Tiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        int numCorrectedMP = 0;
        for (std::map<KeyFrame *, cv::Mat>::iterator mit = CorrectedSE2.begin(), mend = CorrectedSE2.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;
            cv::Mat CorrectedTiw = mit->second;
            cv::Mat NonCorrectedTwi = pKFi->GetPoseInverse();

            std::vector<MapPoint *> vpMPsi = pKFi->GetMapPointMatches();
            for (size_t iMP = 0, endMPi = vpMPsi.size(); iMP < endMPi; iMP++)
            {
                MapPoint *pMPi = vpMPsi[iMP];
                if (!pMPi)
                    continue;
                if (pMPi->mnCorrectedByKF == mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                // std::cout << "CorrectLoop::P3Dw:" << std::endl << P3Dw << std::endl;
                cv::Mat CorrectedP3Dw = CorrectedTiw * NonCorrectedTwi * P3Dw;
                // std::cout << "CorrectLoop::CorrectedP3Dw:" << std::endl << CorrectedP3Dw << std::endl;

                pMPi->SetWorldPos(CorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                numCorrectedMP++;
            }
        }
        // std::cout << "Total corrected map point: " << numCorrectedMP << std::endl;
    }


    // auto start = std::chrono::high_resolution_clock::now();

    // Optimize the essential graph
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, mLoopRT, mTcm, mConfigLoader);
    // auto stop1 = std::chrono::high_resolution_clock::now();
    // auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start);
    // std::cout << "OptimizeEssentialGraph computation time is: " << duration1.count() << " milliseconds." << std::endl;

    // Correct all the map points position based on their first observation.
    // This is also done in Optimizer::GlobalBundleAdjustment.
    // But here we do that for visualization.
    std::vector<MapPoint *> vpMP = mpMap->GetAllMapPoints();
    for (size_t i = 0; i < vpMP.size(); i++)
    {
        MapPoint *pMP = vpMP[i];
        if (pMP->isBad())
            continue;
        if (pMP->nObs < 2)
        {
            mpMap->DeleteMapPoint(pMP);
            continue;
        }

        const std::map<KeyFrame *, size_t> observations = pMP->GetObservations();
        std::map<KeyFrame *, size_t>::const_iterator mit = observations.begin();
        KeyFrame *pKFi = mit->first;
        size_t nKeyPoint = mit->second;
        cv::Point2d image_point = pKFi->mvKeypoints[nKeyPoint].pt;
        cv::Point2d obs_xy = imageFrame2LocalFrame(mConfigLoader.cartImageHeight, mConfigLoader.cartImageWidth,
                                                   mConfigLoader.cartImageScale, image_point);

        cv::Mat LocalPos = cv::Mat(3, 1, CV_32FC1);
        LocalPos.at<float>(0, 0) = obs_xy.x;
        LocalPos.at<float>(1, 0) = obs_xy.y;
        LocalPos.at<float>(2, 0) = 1.0;
        cv::Mat WorldPos = pKFi->GetPose() * LocalPos;
        pMP->SetWorldPos(WorldPos);
    }

    // Add loop edge
    mpCurrentKF->AddLoopEdge(mpMatchedKF);
    mpMatchedKF->AddLoopEdge(mpCurrentKF);

    // If tracking mode is using local window
    if (mConfigLoader.trackMode == 1)
    {
        CorrectLocalWindow();
        // mpTracker->SetLocalised();
    }

    // // Save Keyframe poses
    // std::ofstream ofsKeyFrame;
    // ofsKeyFrame.open("/home/hong/Desktop/PoseError/KeyFrameLoopRelativePose.csv");
    // ofsKeyFrame << "FrameID,tcw_x,tcw_y,tcw_yaw" << std::endl;
    // // Get all the keyframes
    // std::vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
    // for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
    // {
    //     KeyFrame *pKF = vpKFs[i];
    //     cv::Mat pose = pKF->GetPose();
    //     ofsKeyFrame << pKF->mnFrameId << "," << pose.at<float>(0, 2) << "," << pose.at<float>(1, 2) << ","
    //         << atan2(pose.at<float>(1, 0), pose.at<float>(0, 0)) << std::endl;
    // }

    // auto stop2 = std::chrono::high_resolution_clock::now();
    // auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(stop2 - start);
    // std::cout << "CorrectLoop computation time is: " << duration2.count() << " milliseconds." << std::endl;

}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lock(mMutexLoopQueue);
    if (pKF->mnId != 0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexLoopQueue);
    return (!mlpLoopKeyFrameQueue.empty());
}

void LoopClosing::CorrectLocalWindow()
{
    /** For local window, to update local features world position and frames position */
    // Correct the local window frames position. The poses are propogated from associated keyframe.
    std::vector<Frame *> vpLocalFrames = mpTracker->GetLocalWindowFrames();
    Frame *pFend = vpLocalFrames[vpLocalFrames.size() - 1];
    cv::Mat NonCorrectedTwj = pFend->GetPoseInverse();
    cv::Mat CorrectedTjw = mpCurrentKF->GetPose();
    // std::cout << "mpCurrentKF->mnFrameId: " << mpCurrentKF->mnFrameId << std::endl;
    // std::cout << "pFend->mnFrameId: " << pFend->mnId << std::endl;
    for (int i = 0; i < vpLocalFrames.size(); i++)
    {
        Frame *pFi = vpLocalFrames[i];
        cv::Mat Tiw = pFi->GetPose();
        // std::cout << "Frame ID: " << pFi->mnId << std::endl
        //           << "original Pose: " << std::endl
        //           << pFi->GetPose() << std::endl;

        cv::Mat correctedPose = CorrectedTjw * NonCorrectedTwj * Tiw;
        pFi->SetPose(correctedPose);
        // std::cout << " corrected Pose: " << std::endl
        //           << correctedPose
        //           << std::endl;
    }

    /** Retreive all the features observed within the window, 
     * correct its world position with motion compensation */

    /** Retreive all the features observed within the window, correct its world position*/
    for (int i = 0; i < vpLocalFrames.size(); i++)
    {
        Frame *pFi = vpLocalFrames[i];
        int frameId = pFi->mnId;
        std::vector<Feature *> vFeatures = pFi->mvFeatures;
        std::vector<cv::Point2f> vFeatureCoordinates = pFi->mvFeatureCoordinates;

        for (int j = 0; j < vFeatures.size(); j++)
        {
            Feature *pFTj = vFeatures[j];
            cv::Point2d image_point = vFeatureCoordinates[j];
            if (pFTj->mnFirstFrame == frameId)
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
                double frameTimeDiff = pFi->mTimestamp - pFi->mPreviousTimestamp;
                double deltaT = frameTimeDiff * (factorT - factorT0);

                // Correct the local observation using optimized velocity
                cv::Mat T0t = cv::Mat::eye(3, 3, CV_32F); // From t=0 to t=t'
                Eigen::Vector3d optimizedVelocity = pFi->GetVelocity();
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
                cv::Mat WorldPos = pFi->GetPose() * correctedLocalPos;

                // Set the updated world position
                pFTj->SetWorldPos(WorldPos);
            }
        }
    }

    /** Only keep and correct the last frame in the window */
    // // Clear the local window and only keep the last frame
    // mpTracker->ClearLocalWindow();
    // std::vector<Frame *> vpLocalFrames = mpTracker->GetLocalWindowFrames();
    // // std::cout << "vpLocalFrames size: " << vpLocalFrames.size() << std::endl;
    // Frame *pFi = vpLocalFrames[0];
    // KeyFrame *pReferenceKeyFrame = pFi->GetReferenceKeyFrame();
    // cv::Mat correctedPose = pReferenceKeyFrame->GetPose() * pFi->GetRT();
    // pFi->SetPose(correctedPose);
    // std::cout << "Frame " << pFi->mnId << " corrected Pose: " << std::endl
    //           << correctedPose << std::endl;

    // // Retreive all the features observed within current frame, correct its world position
    // int frameId = pFi->mnId;
    // std::vector<Feature *> vFeatures = pFi->mvFeatures;
    // std::vector<cv::Point2f> vFeatureCoordinates = pFi->mvFeatureCoordinates;

    // for (int j = 0; j < vFeatures.size(); j++)
    // {
    //     Feature *pFTj = vFeatures[j];
    //     cv::Point2d image_point = vFeatureCoordinates[j];

    //     cv::Point2d ptLocal = imageFrame2LocalFrame(mConfigLoader.cartImageHeight, mConfigLoader.cartImageWidth,
    //                                                 mConfigLoader.cartImageScale, image_point);
    //     cv::Mat LocalPos = cv::Mat(3, 1, CV_32FC1);
    //     LocalPos.at<float>(0, 0) = ptLocal.x;
    //     LocalPos.at<float>(1, 0) = ptLocal.y;
    //     LocalPos.at<float>(2, 0) = 1.0;
    //     cv::Mat WorldPos = pFi->GetPose() * LocalPos;
    //     pFTj->SetWorldPos(WorldPos);
    // }
}

void LoopClosing::SetFinish()
{
    std::unique_lock<std::mutex> lock(mMutexFinish);
    mbFinished = true;
}