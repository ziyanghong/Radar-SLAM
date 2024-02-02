#include "FeatureTracker.h"
#include "Optimizer.h"

/* To visualize the tracking points*/
// Create a mask image for drawing purposes
cv::Mat mask;
// Create some random colors
std::vector<cv::Scalar> colors;
cv::RNG rng;
float circleSize = 2.0;

/* Constructor */
FeatureTracker::FeatureTracker(const ConfigLoader &config_loader)
{
    // Make Grids
    float rowRange = (config_loader.cartImageHeight - 1) / config_loader.numberGridRow;
    float colRange = (config_loader.cartImageWidth - 1) / config_loader.numberGridCol;
    for (int i = 0; i < config_loader.numberGridRow; i++)
    {
        float y1 = rowRange * i;
        float y2 = rowRange * (i + 1);
        for (int j = 0; j < config_loader.numberGridCol; j++)
        {
            float x1 = colRange * j;
            float x2 = colRange * (j + 1);
            Grid grid;
            grid.push_back(x1);
            grid.push_back(y1);
            grid.push_back(x2);
            grid.push_back(y2);
            // std::cout << "grid: " << grid[0] << "," << grid[1] << "," << grid[2] << "," << grid[3] << std::endl;
            mvGrids.push_back(grid);
        }
    }
    mvNumPointsInGrids.resize(mvGrids.size());
}

bool FeatureTracker::Initialized(Frame *pFrame, const ConfigLoader &config_loader)
{

    cv::Mat cart_image = pFrame->mCartImage.clone();
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    // Set Velocity
    Eigen::Matrix<double, 3, 1> velocity(0.0, 0.0, 0.0);
    pFrame->SetVelocity(velocity);

    FeatureExtractionANMS(cart_image, config_loader, keypoints, descriptors);
    for (int i = 0; i < keypoints.size(); ++i)
    {
        pFrame->mvFeatureCoordinates.push_back(keypoints[i].pt);
    }
    pFrame->SetKeypoints(keypoints);
    pFrame->SetDescriptors(descriptors);

    cv::Mat img_keypoints;
    cv::drawKeypoints(cart_image, keypoints, img_keypoints);
    // resize it
    cv::resize(img_keypoints, img_keypoints, cv::Size(800, 800), cv::INTER_NEAREST);
    pFrame->mKeyPointsImage = img_keypoints;

    // Set tracked feature in frame
    std::vector<Feature *> features;
    for (int i = 0; i < keypoints.size(); i++)
    {

        cv::KeyPoint keypt = keypoints[i];
        cv::Point2d ptLocal = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth,
                                                    config_loader.cartImageScale, keypt.pt);
        cv::Mat LocalPos = cv::Mat(3, 1, CV_32FC1);
        LocalPos.at<float>(0, 0) = ptLocal.x;
        LocalPos.at<float>(1, 0) = ptLocal.y;
        LocalPos.at<float>(2, 0) = 1.0;
        cv::Mat WorldPos = pFrame->GetPose() * LocalPos;
        Feature *pFT = new Feature(WorldPos, pFrame);
        pFT->AddObservation(pFrame->mnId, i);
        // std::cout << "pFrame mnId: " << pFrame->mnId << std::endl;
        // std::cout << "pFT.observations.size() is " << pFT->mObservations.size() << '\n';

        // Insert feature into current frame
        pFrame->mvFeatures.push_back(pFT);

        // Insert feature into local window
        mspFeatures.insert(pFT);
    }

    std::vector<Feature *> NewFeatures;
    std::vector<cv::Point2f> NewFeatureCoordinates;

    // cv::Mat imageTrackingVisualization;
    // DrawFeatures(pFrame->mCartImage,
    //              NewFeatures,
    //              NewFeatureCoordinates,
    //              pFrame->mvFeatures,
    //              pFrame->mvFeatureCoordinates,
    //              imageTrackingVisualization);
    // std::string filename = "/home/hong/Desktop/trackPoints/" + std::to_string(pFrame->mnId) + ".jpg";
    // cv::imwrite(filename, imageTrackingVisualization);

    // // To visualize tracking points
    // cv::Mat frame;
    // cv::cvtColor(cart_image, frame, CV_GRAY2BGR);
    // mask = cv::Mat::zeros(frame.size(), frame.type());
    // for (int i = 0; i < 100000; i++)
    // {
    //     int r = rng.uniform(0, 256);
    //     int g = rng.uniform(0, 256);
    //     int b = rng.uniform(0, 256);
    //     colors.push_back(cv::Scalar(r, g, b));
    // }
    // for (int i = 0; i < pFrame->mvFeatures.size(); i++)
    // {
    //     Feature *pFT = pFrame->mvFeatures[i];

    //     // draw the points
    //     cv::circle(frame, keypoints[i].pt, circleSize, colors[pFT->mnId], -1);
    // }
    // cv::Mat img;
    // cv::add(frame, mask, img);
    // std::string filename = "/home/hong/Desktop/trackPoints/" + std::to_string(pFrame->mnId) + ".jpg";
    // cv::imwrite(filename, img);
    pFrame->ComputeGlobalDescriptor();

    if (pFrame->mnId >= 1)
    {
        std::cout << "Initialized FeatureTracker." << std::endl;
        return true;
    }
    else
    {
        return false;
    }
}

void FeatureTracker::TrackNewFrame(Frame *pNewFrame, Frame *pLastFrame, std::vector<Frame *> vpWindowFrames,
                                   std::vector<cv::Mat> vRT, cv::Mat &RT,
                                   const ConfigLoader &config_loader)
{

    // Track valid feature respect to last frame
    cv::Mat img1 = pLastFrame->mCartImage;
    cv::Mat img2 = pNewFrame->mCartImage;

    std::vector<cv::Point2f> p1, p2;
    std::vector<cv::DMatch> matches;
    p1 = pLastFrame->mvFeatureCoordinates;
    // std::cout << "p1.size: " << p1.size() << std::endl;
    // cv::Mat imageTrackingVisualization;
    // DrawFeatures(lastFrame.mCartImage, lastFrame.mvFeatures, lastFrame.mvFeatureCoordinates, imageTrackingVisualization);

    // cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    // cv::imshow( "Display window", img2 );                   // Show our image inside it.
    // cv::waitKey(0);                                          // Wait for a keystroke in the window

    std::vector<uchar> status;
    std::vector<float> err;
    const float maxSpeedMeterSecond = 40; // m/s
    int windowSize = 40 * 0.25 / config_loader.cartImageScale * (config_loader.cartImageWidth / 2);
    cv::Size win = cv::Size(windowSize, windowSize);
    int maxLevel = 0;
    // auto start = std::chrono::high_resolution_clock::now();
    // cv::calcOpticalFlowPyrLK(img1, img2, p1, p2, status, err, win, maxLevel);
    cv::calcOpticalFlowPyrLK(img1, img2, p1, p2, status, err);


    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // std::cout << "OpticalFlowPyrLK computation time is: " << duration.count() << " milliseconds." << std::endl;
    for (int i = 0; i < p1.size(); i++)
    {
        if (status[i] == 1)
        {
            matches.push_back(cv::DMatch(i, i, 0.0));
        }
    }

    // Remove moving targets
    DynamicTargetRemoval(config_loader, matches, p1, p2);

    std::vector<cv::Point2f> prev, curr;
    for (int i = 0; i < matches.size(); i++)
    {
        prev.push_back(p1[matches[i].trainIdx]);
        curr.push_back(p2[matches[i].queryIdx]);
    }
    mnTrackedPoints = matches.size();

    // // To visualize tracking points after outliers rejection
    // const float font_size = 0.3;
    // cv::Mat frame; 
    // cv::cvtColor(img2, frame, CV_GRAY2BGR);
    // if (pNewFrame->mnId % 20 == 0)
    // {
    //     mask = cv::Mat::zeros(frame.size(), frame.type());
    // }
    // for (int i = 0; i < matches.size(); i++)
    // {
    //     Feature *pFT = pLastFrame->mvFeatures[matches[i].trainIdx];

    //     // draw the tracks
    //     cv::line(mask, p2[matches[i].queryIdx], p1[matches[i].trainIdx], colors[pFT->mnId], 1);
    //     cv::circle(frame, p2[matches[i].queryIdx], circleSize, colors[pFT->mnId], -1);
    //     cv::putText(frame, std::to_string(pFT->mnId), p2[matches[i].queryIdx],
    //                 cv::FONT_HERSHEY_SIMPLEX, font_size, colors[pFT->mnId]);
    // }
    // cv::Mat img;
    // cv::add(frame, mask, img);
    // std::string filename = "/home/hong/Desktop/trackPoints/" + std::to_string(pNewFrame->mnId) + ".jpg";
    // cv::imwrite(filename, img);

    // Compute relative motion
    bool bOK;
    RT = estimateMotionICP(config_loader, prev, curr, bOK);
    // std::cout << "RT between current frame and last frame: " << std::endl
    //           << RT << std::endl;
    // std::cout << "Initial guess of pose of current frame: " << std::endl
    //           << pLastFrame->GetPose() * RT << std::endl;
    // Insert the new RT in the vector of RTs for optimization
    std::vector<cv::Mat> vRT4Optimization = vRT;
    vRT4Optimization.push_back(RT);
    // Update current pose
    cv::Mat pose = pLastFrame->GetPose() * RT;
    pNewFrame->SetPose(pose);
    // std::cout << "Current yaw: " << atan2(pose.at<float>(1, 0), pose.at<float>(0, 0)) * 180 / 3.141592
    //           << " in degrees before motion compensation." << std::endl;

    // Set velocity vector of current frame, compute velocity
    Eigen::Matrix<double, 3, 1> velocity;
    velocity(0) = RT.at<float>(0, 2) / (pNewFrame->mTimestamp - pLastFrame->mTimestamp);
    velocity(1) = RT.at<float>(1, 2) / (pNewFrame->mTimestamp - pLastFrame->mTimestamp);
    velocity(2) = atan2(RT.at<float>(1, 0), RT.at<float>(0, 0)) / (pNewFrame->mTimestamp - pLastFrame->mTimestamp);
    pNewFrame->SetVelocity(velocity);

    // // Initialize velocity vector with previous velocity.
    // pNewFrame->SetVelocity(pLastFrame->GetVelocity());

    // Associate the features observed in the current frame with the local window features
    std::vector<Feature *> featuresLastFrame = pLastFrame->mvFeatures;
    pNewFrame->mvFeatureCoordinates = curr;
    std::vector<Feature *> vpTrackedFeatures;
    std::vector<cv::Point2f> trackedFeatureCoordinates = curr;
    std::vector<Feature *> vpNewFeature;
    std::vector<cv::Point2f> vNewFeatureCoordinates;
    for (int i = 0; i < matches.size(); i++)
    {
        Feature *pFT = featuresLastFrame[matches[i].trainIdx];
        pNewFrame->mvFeatures.push_back(pFT);
        pFT->AddObservation(pNewFrame->mnId, i);
        vpTrackedFeatures.push_back(pFT);
        // std::cout << "pFT.observations.size() is " << pFT->mObservations.size() << '\n';
    }
    mvpTrackedFeatures = vpTrackedFeatures;

    /* Use all the associated features to perform optimization to refine current pose. */
    vpWindowFrames.push_back(pNewFrame);
    // Optimizer::OptimizeTracking(pNewFrame, vpWindowFrames, vRT4Optimization, config_loader);
    Optimizer::OptimizeTrackingMotionPrior(pNewFrame, pLastFrame, vpWindowFrames, config_loader);
    // std::cout << "Current pose after optimizeTracking: " << std::endl
    //           << pNewFrame->GetPose() << std::endl;

    pose = pNewFrame->GetPose();
    // std::cout << "Current yaw: " << atan2(pose.at<float>(1, 0), pose.at<float>(0, 0)) * 180 / 3.141592
    //           << " in degrees after motion compensation." << std::endl;

    // Optimizer::OptimizeTrackPoints(pNewFrame, pLastFrame, vpWindowFrames, config_loader);


    // Check number of visible points, if less then a number, generate new feature
    if (curr.size() < config_loader.minimumTrackPoints)
    {
        // std::cout << "Tracked points below threshold, needs to generate new points."
        //           << " Current tracke points number: " << curr.size() << std::endl;
        cv::Mat cart_image = pNewFrame->mCartImage;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        mspNewFeatures.clear();

        // Extract new keypoints
        FeatureExtractionANMS(cart_image, config_loader, keypoints, descriptors);

        // Add the best candidates points
        std::vector<cv::Point2f> candidates;
        for (unsigned int i = 0; i < keypoints.size(); i++)
        {
            candidates.push_back(keypoints[i].pt);
        }

        int maxAddedFeature = config_loader.maximumTrackPoints - curr.size();
        std::set<unsigned int> newFeatureIDs = AddBestCandidates(curr, candidates, maxAddedFeature);

        // Insert new feature points
        Eigen::Vector3d optimizedVelocity = pNewFrame->GetVelocity();
        for (std::set<unsigned int>::iterator sit = newFeatureIDs.begin(), send = newFeatureIDs.end(); sit != send; sit++)
        {
            unsigned int i = *sit;

            // Create new Feature
            cv::Point2f pt = candidates[i];
            cv::Point2d ptLocal = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth,
                                                        config_loader.cartImageScale, pt);
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
            double frameTimeDiff = pNewFrame->mTimestamp - pNewFrame->mPreviousTimestamp;
            double deltaT = frameTimeDiff * (factorT - factorT0);
            // std::cout << "deltaT: " << deltaT << std::endl << std::endl;

            // Correct the local observation using optimized velocity
            cv::Mat T0t = cv::Mat::eye(3, 3, CV_32F); // From t=0 to t=t'
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
            cv::Mat WorldPos = pNewFrame->GetPose() * correctedLocalPos;

            // Add feature Coordinates and Feature vector in the current frame
            Feature *pFT = new Feature(WorldPos, pNewFrame);
            pNewFrame->mvFeatureCoordinates.push_back(pt);
            pNewFrame->mvFeatures.push_back(pFT);
            size_t featureIdxInFrame = pNewFrame->mvFeatures.size() - 1;
            pFT->AddObservation(pNewFrame->mnId, featureIdxInFrame);
            mspFeatures.insert(pFT);
            vpNewFeature.push_back(pFT);
            vNewFeatureCoordinates.push_back(pt);

            mspNewFeatures.insert(pFT);
        }
    }



    // // Visualize tracked points in current frame
    // cv::Mat imageTrackingVisualization;
    // DrawFeatures(pNewFrame->mCartImage,
    //              vpTrackedFeatures,
    //              trackedFeatureCoordinates,
    //              vpNewFeature,
    //              vNewFeatureCoordinates,
    //              imageTrackingVisualization);
    // std::string filename = "/home/hong/Desktop/trackPoints/" + std::to_string(pNewFrame->mnId) + ".jpg";
    // cv::imwrite(filename, imageTrackingVisualization);


    // std::cout << "End of TrackNewFrame" << std::endl;
}

std::set<Feature *> FeatureTracker::GetAllFeatures()
{
    return mspFeatures;
}

void FeatureTracker::DrawFeatures(cv::Mat image,
                                  std::vector<Feature *> vpTrackedFeatures,
                                  std::vector<cv::Point2f> trackedFeatureCoordinates,
                                  std::vector<Feature *> vpNewFeature,
                                  std::vector<cv::Point2f> vNewFeatureCoordinates,
                                  cv::Mat &visImage)
{
    const float font_size = 0.5;
    const int circle_size = 2;
    cv::Mat imageTrackingVisualizationLast = image;
    cv::cvtColor(imageTrackingVisualizationLast, imageTrackingVisualizationLast, cv::COLOR_GRAY2RGB);
    // Draw old points
    for (int i = 0; i < vpTrackedFeatures.size(); i++)
    {
        cv::circle(imageTrackingVisualizationLast, trackedFeatureCoordinates[i], circle_size, cv::Scalar(0, 255, 255));
        cv::putText(imageTrackingVisualizationLast, std::to_string(vpTrackedFeatures[i]->mnId), trackedFeatureCoordinates[i],
                    cv::FONT_HERSHEY_SIMPLEX, font_size, cv::Scalar(0, 255, 255));
    }
    // cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
    // cv::imshow( "Display window", imageTrackingVisualizationLast );                   // Show our image inside it.
    // cv::waitKey(0);                                          // Wait for a keystroke in the window

    // Draw new points
    for (int i = 0; i < vpNewFeature.size(); i++)
    {
        cv::circle(imageTrackingVisualizationLast, vNewFeatureCoordinates[i], circle_size, cv::Scalar(0, 255, 0));
        cv::putText(imageTrackingVisualizationLast, std::to_string(vpNewFeature[i]->mnId), vNewFeatureCoordinates[i],
                    cv::FONT_HERSHEY_SIMPLEX, font_size, cv::Scalar(0, 255, 0));
    }

    visImage = imageTrackingVisualizationLast.clone();
}

std::set<unsigned int> FeatureTracker::AddBestCandidates(std::vector<cv::Point2f> trackedPoints,
                                                         std::vector<cv::Point2f> candidates,
                                                         int maxAddedFeature)
{
    const int maxPointsInGrid = 1;
    const int minPixelDistance = 30;
    std::set<unsigned int> newFeatureIDs;
    std::multimap<Grid *, cv::Point2f> multimapPointsInGrids;
    typedef std::multimap<Grid *, cv::Point2f>::iterator MMAPIterator;

    // Count number of tracked feature in each grid.
    int totalPointsInGrid = 0;
    for (int i = 0; i < mvGrids.size(); i++)
    {
        mvNumPointsInGrids[i] = 0;
        Grid grid = mvGrids[i];
        for (int j = 0; j < trackedPoints.size(); j++)
        {
            cv::Point2f pt = trackedPoints[j];
            if (isInsideGrid(grid, pt))
            {
                totalPointsInGrid++;
                mvNumPointsInGrids[i] = mvNumPointsInGrids[i] + 1;
                multimapPointsInGrids.insert(std::pair<Grid *, cv::Point2f>(&grid, pt));
            }
        }
        // std::cout << "mvNumPointsInGrids[i]: " << mvNumPointsInGrids[i] << std::endl;
    }
    // std::cout << "trackPoints number: " << trackedPoints.size() << " , totalPointsInGrid: " << totalPointsInGrid << std::endl;

    // Sort grid based on numnber in each grid
    SortGrids();

    // Incrementally add new features
    int addedCount = 0;
    for (int i = 0; i < mvGrids.size(); i++)
    {
        Grid grid = mvGrids[i];
        for (int j = 0; j < candidates.size(); j++)
        {
            cv::Point2f pt = candidates[j];

            if (isInsideGrid(grid, pt))
            {
                // Check if the grid is empty
                if (mvNumPointsInGrids[i] == 0)
                {
                    newFeatureIDs.insert(j);
                    addedCount++;
                    mvNumPointsInGrids[i] = mvNumPointsInGrids[i] + 1;
                    multimapPointsInGrids.insert(std::pair<Grid *, cv::Point2f>(&grid, pt));
                }

                if (mvNumPointsInGrids[i] > 0 && mvNumPointsInGrids[i] < maxPointsInGrid)
                {
                    // Iterate through all the points in the grid, find the minimum
                    std::pair<MMAPIterator, MMAPIterator> values = multimapPointsInGrids.equal_range(&grid);
                    float minDist = 100;
                    for (MMAPIterator it = values.first; it != values.second; it++)
                    {
                        cv::Point2f ptInGrid = it->second;
                        cv::Point2f diff = ptInGrid - pt;
                        float dis = cv::sqrt(diff.x * diff.x + diff.y * diff.y);
                        // std::cout << "ptInGrid: " << ptInGrid << std::endl;
                        // std::cout << "pt: " << pt << std::endl;
                        // std::cout << "dis: " << dis << std::endl;

                        if (dis < minDist)
                        {
                            minDist = dis;
                        }
                    }

                    if (minDist > minPixelDistance)
                    {
                        // std::cout << "point x: " << pt.x << " point y: " << pt.y << std::endl;
                        // std::cout << "grid: " << grid[0] << "," << grid[1] << "," << grid[2] << "," << grid[3] << std::endl;
                        // std::cout << "Minimum disttance: " << minDist << " between the point to other points" << std::endl;
                        newFeatureIDs.insert(j);
                        addedCount++;
                        mvNumPointsInGrids[i] = mvNumPointsInGrids[i] + 1;
                        multimapPointsInGrids.insert(std::pair<Grid *, cv::Point2f>(&grid, pt));
                    }
                }
            }
        }

        if (addedCount >= maxAddedFeature)
        {
            // std::cout << "addedCount: " << addedCount << std::endl;
            break;
        }
    }

    return newFeatureIDs;
}

bool FeatureTracker::isInsideGrid(Grid grid, cv::Point2f point)
{
    float x1 = grid[0];
    float y1 = grid[1];
    float x2 = grid[2];
    float y2 = grid[3];

    if (point.x > x1 and point.x < x2 and point.y > y1 and point.y < y2)
        return true;

    // std::cout << "point x: " << point.x << " point y: " << point.y << std::endl;
    // std::cout << "grid: " << grid[0] << "," << grid[1] << "," << grid[2] << "," << grid[3] << std::endl;
    return false;
}

void FeatureTracker::SortGrids()
{
    // std::cout << "Sort Grids" << std::endl;
    std::vector<int> Indx(mvGrids.size());
    std::iota(std::begin(Indx), std::end(Indx), 0);
    cv::sortIdx(mvNumPointsInGrids, Indx, CV_SORT_ASCENDING); //  Sort in ASCENDING order
    std::vector<Grid> GridsSorted;
    std::vector<int> NumPointsInGridsSorted;
    for (unsigned int i = 0; i < mvGrids.size(); i++)
    {
        GridsSorted.push_back(mvGrids[Indx[i]]);
        NumPointsInGridsSorted.push_back(mvNumPointsInGrids[Indx[i]]);
        // std::cout << "NumPointsInGridsSorted: " << NumPointsInGridsSorted[i] << std::endl;
    }

    mvGrids = GridsSorted;
    mvNumPointsInGrids = NumPointsInGridsSorted;
}

std::set<Feature *> FeatureTracker::GetNewFeatures()
{
    return mspNewFeatures;
}

std::vector<Feature *> FeatureTracker::GetTrackedFeatures()
{
    return mvpTrackedFeatures;
}


FeatureTracker::~FeatureTracker()
{}