#include "KeyFrame.h"
#include "MapPoint.h"
#include <omp.h>

// namespace ROAM
// {
/* Function Definitions */
static double argInit_real_T()
{
    return 0.0;
}

long unsigned int KeyFrame::nNextId = 0;
KeyFrame::KeyFrame(Frame F, Map *pMap) : mnFrameId(F.mnId), mTimestamp(F.mTimestamp), mCartImage(F.mCartImage), 
                                         mvFeatures(F.mvFeatures), mvFeatureCoordinates(F.mvFeatureCoordinates), 
                                         mPreviousTimestamp(F.mPreviousTimestamp), mVelocity(F.mVelocity),
                                         mConfigLoader(F.config_loader), mnMatchesWithLastKeyFrame(F.mnMatchesWithLastKeyFrame),
                                         mPolarImage(F.mPolarImage), mpParent(NULL), mpMap(pMap), mbBad(false)
{
    // Keyframe id(not the same as frame id)
    mnId = nNextId++;
    mvKeypoints = F.mvKeypoints;
    // std::cout << "Keyframe constructor, mvKeypoints.size: " << mvKeypoints.size() << std::endl;
    mvDescriptors = F.mvDescriptors.clone();

    int N = mvKeypoints.size();
    mvpMapPoints = std::vector<MapPoint *>(N, static_cast<MapPoint *>(NULL));
    mvpMapPoints = F.mvpMapPoints;

    SetPose(F.GetPose());
    SetM2dpFeature(F.GetFeature());
    SetPointcloud(F.mvPointCloud.clone());

    int index = 0;
    for (std::vector<MapPoint *>::iterator vit = mvpMapPoints.begin(), vend = mvpMapPoints.end(); vit != vend; vit++)
    {
        MapPoint *pMP = *vit;

        if (!pMP)
        {
            index++;
        }
    }
}

KeyFrame::KeyFrame()
{
}

void KeyFrame::SetM2dpFeature(const cv::Mat &Feature_)
{
    Feature_.copyTo(mFeature);
    // mFeature.convertTo(mFeature, CV_64F);
    // std::cout << "mFeature rows: " << mFeature.rows << " mFeature cols: " << mFeature.cols << std::endl;
    // std::cout << "SetM2dpFeature " <<std::endl;
}

void KeyFrame::SetPointcloud(const cv::Mat &Pointcloud_)
{
    Pointcloud_.copyTo(mvPointCloud);
    // std::cout << "mvPointCloud rows: " << mvPointCloud.rows << " mvPointCloud cols: " << mvPointCloud.cols << std::endl;

    // std::cout << "SetPointcloud " <<std::endl;

}

void KeyFrame::SetPose(const cv::Mat &Tcw_)
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    Tcw_.copyTo(Tcw);
    cv::Mat Rcw = Tcw.rowRange(0, 2).colRange(0, 2);
    cv::Mat tcw = Tcw.rowRange(0, 2).col(2);
    cv::Mat Rwc = Rcw.t();
    cv::Mat Ow = -Rwc * tcw;

    Twc = cv::Mat::eye(3, 3, Tcw.type());
    Rwc.copyTo(Twc.rowRange(0, 2).colRange(0, 2));
    Ow.copyTo(Twc.rowRange(0, 2).col(2));
}

bool KeyFrame::isBad()
{
    return mbBad;
}

Eigen::Matrix<double, 3, 1> KeyFrame::GetVelocity()
{
    return mVelocity;
}

cv::Mat KeyFrame::GetPose()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Tcw.clone();
}

cv::Mat KeyFrame::GetPoseInverse()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Twc.clone();
}

cv::Mat KeyFrame::GetRotation()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Tcw.rowRange(0, 2).colRange(0, 2).clone();
}

cv::Mat KeyFrame::GetTranslation()
{
    std::unique_lock<std::mutex> lock(mMutexPose);
    return Tcw.rowRange(0, 2).col(2).clone();
}

void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t &idx)
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    mvpMapPoints[idx] = pMP;
}

std::vector<MapPoint *> KeyFrame::GetMapPointMatches()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mvpMapPoints;
}

void KeyFrame::SetNumberMatchesWithLoop(int n)
{
    mnMatchesToLoop = n;
}

int KeyFrame::GetNumberMatchesWithLoop()
{
    return mnMatchesToLoop;
}

int KeyFrame::GetNumberMatchesWithLastKeyFrame()
{
    return mnMatchesWithLastKeyFrame;
}

std::vector<KeyFrame *> KeyFrame::GetVectorCovisibleKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexConnections);
    // std::cout << "KeyFrame::GetVectorCovisibleKeyFrames():: return number of mvpConnectedKeyFrames is " << mvpConnectedKeyFrames.size() << std::endl;
    return mvpConnectedKeyFrames;
}

void KeyFrame::UpdateConnections()
{
    std::map<KeyFrame *, size_t> KFcounter;

    std::vector<MapPoint *> vpMP;

    {
        std::unique_lock<std::mutex> lockMPs(mMutexFeatures);
        vpMP = mvpMapPoints;
    }

    //For all map points in keyframe check in which other keyframes are they seen
    //Increase counter for those keyframes
    for (std::vector<MapPoint *>::iterator vit = vpMP.begin(), vend = vpMP.end(); vit != vend; vit++)
    {
        MapPoint *pMP = *vit;

        if (!pMP)
            continue;

        if (pMP->isBad())
            continue;

        std::map<KeyFrame *, size_t> observations = pMP->GetObservations();
        // std::cout << "pMP observations: " << observations.size() << std::endl;
        for (std::map<KeyFrame *, size_t>::iterator mit = observations.begin(); mit != observations.end(); mit++)
        {
            if (mit->first->mnId == mnId)
                continue;
            KFcounter[mit->first]++;
        }
    }

    // This should not happen except for the first keyframe
    if (KFcounter.empty())
        return;

    // std::cout << "KeyFrame::UpdateConnections(): KFcounter is not empty." << std::endl;

    const int minMapPointsContains = 2;
    for (std::map<KeyFrame *, size_t>::iterator mit = KFcounter.begin(); mit != KFcounter.end(); mit++)
    {

        // std::cout << "KeyFrame::UpdateConnections(): KFcounter[mit->first]: " << KFcounter[mit->first] << std::endl;
        if (KFcounter[mit->first] >= minMapPointsContains)
        {
            mvpConnectedKeyFrames.push_back(mit->first);
        }
    }

    if (mvpConnectedKeyFrames.size() == 0)
        std::cout << "KeyFrame::UpdateConnections(): mvpConnectedKeyFrames is empty." << std::endl
                  << "This should not happen. " << std::endl;
}

void KeyFrame::ComputeGlobalDescriptor()
{
    /**
     * M2DP global descriptor extraxtion
    */
    // auto start = std::chrono::high_resolution_clock::now();

    cv::Mat PolarImage;
    double max_selected_distance = mConfigLoader.cartImageScale;
    double radar_resolution;
    double max_distance;

    // Select parameters based on dataset
    if (mPolarImage.cols > 1000)
    {
        // Oxford radar
        PolarImage = mPolarImage.rowRange(0, mPolarImage.rows).colRange(11, mPolarImage.cols);
        // std::cout << "PolarImage.cols: " << PolarImage.cols << " rows: " << PolarImage.rows << std::endl;
        radar_resolution = 0.0432;
        max_distance = 163.0;
    }
    else
    {
        // Our RADIATE data
        PolarImage = mPolarImage;
        radar_resolution = 0.1731;
        max_distance = 100.0;
    }

    int rows = PolarImage.rows;
    int cols = PolarImage.cols;
    emxArray_uint8_T *img = emxCreate_uint8_T(rows, cols);
#pragma omp parallel for   
    /* Loop over the array to initialize each element of polar image. */
    for (int idx0 = 0; idx0 < img->size[0U]; idx0++)
    {
        for (int idx1 = 0; idx1 < img->size[1U]; idx1++)
        {
            unsigned char v = PolarImage.at<uchar>(idx0, idx1);
            img->data[idx0 + img->size[0] * idx1] = v;
        }
    }

    double featureForward[192]; 
    // double featureReverse[192];
    emxArray_real_T *point_cloud_forward;
    // emxArray_real_T *point_cloud_reverse;
    emxInitArray_real_T(&point_cloud_forward, 2);
    // emxInitArray_real_T(&point_cloud_reverse, 2);

    /* Call the entry-point 'generateGlobalFeature' forward and reverse feature generation. */
    // generateGlobalFeature(img,
    //                       max_selected_distance,
    //                       radar_resolution,
    //                       max_distance,
    //                       featureForward,
    //                       point_cloud_forward,
    //                       featureReverse,
    //                       point_cloud_reverse);

    /* Call the entry-point 'generateGlobalFeature'. */
    generateGlobalFeature(img,
                          max_selected_distance,
                          radar_resolution,
                          max_distance,
                          featureForward,
                          point_cloud_forward);

    cv::Mat featureMat = cv::Mat::zeros(1, 192, CV_64F);
    // cv::Mat featureMatReverse = cv::Mat::zeros(1, 192, CV_64F);

    for (int i = 0; i < 192; i++)
    {
        featureMat.at<double>(0, i) = featureForward[i];
        // featureMatReverse.at<double>(0,i) = featureReverse[i];
    }
    double *data = point_cloud_forward->data;
    int rowsPointCloud = point_cloud_forward->size[0];
    cv::Mat mat_point_cloud = cv::Mat::ones(rowsPointCloud, 2, CV_64F);

    /* Loop over the array to retrieve each element. */
#pragma omp parallel for 
    for (int idx0 = 0; idx0 < point_cloud_forward->size[0]; idx0++)
    {

        for (int idx1 = 0; idx1 < 2; idx1++)
        {
            double v = point_cloud_forward->data[idx0 + point_cloud_forward->size[0] * idx1];
            mat_point_cloud.at<double>(idx0, idx1) = v;
        }
    }

    // Need to rotate the points 90 degree to match our local coordinate frame setting
    cv::Mat rotate90 = cv::Mat::zeros(2, 2, CV_64F);
    double sinA = -1;
    double cosA = 0;
    rotate90.at<double>(0, 0) = cosA;
    rotate90.at<double>(0, 1) = -sinA;
    rotate90.at<double>(1, 0) = sinA;
    rotate90.at<double>(1, 1) = cosA;
    cv::Mat rotate_point_cloud = rotate90 * mat_point_cloud.t();
    cv::Mat rotate_point_cloud_transpose = rotate_point_cloud.t();

    // std::cout << "featureMat rows: " << featureMat.rows << " featureMat cols: " << featureMat.cols << std::endl;
    // std::cout << "size[0]: " << point_cloud_forward->size[0] << " size[1]: " << point_cloud_forward->size[1] << std::endl;
    // std::cout << "featureMat:" <<std::endl;
    // std::cout << featureMat << std::endl;
    mFeature = featureMat.clone();
    // mFeatureReverse = featureMatReverse.clone();
    mvPointCloud = rotate_point_cloud_transpose.clone();

    // Destroy matlab array
    emxDestroyArray_uint8_T(img);
    emxDestroyArray_real_T(point_cloud_forward);

    /* Terminate the application.
     You do not need to do this more than one time. */
    generateGlobalFeature_terminate();

    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // std::cout << "M2DP computation time is: " << duration.count() << " milliseconds." << std::endl;
}

cv::Mat KeyFrame::GetFeature()
{
    return mFeature.clone();
}

cv::Mat KeyFrame::GetFeatureReverse()
{
    return mFeatureReverse.clone();
}

// // Using FLIRT library to detect Interest Points
// void KeyFrame::DetectInterestPoints()
// {

//     unsigned int scale = 5, dmst = 2, window = 3, detectorType = 0, descriptorType = 0, distanceType = 2, strategy = 0;
//     double baseSigma = 0.2, sigmaStep = 1.4, minPeak = 0.34, minPeakDistance = 0.001, success = 0.95, inlier = 0.4, matchingThreshold = 0.4;
//     bool useMaxRange = false;
//     SimpleMinMaxPeakFinder *m_peakMinMax = new SimpleMinMaxPeakFinder(minPeak, minPeakDistance);

//     Detector *m_detector = NULL;
//     CurvatureDetector *m_detectorCurvature = NULL;
//     m_detectorCurvature = new CurvatureDetector(m_peakMinMax, scale, baseSigma, sigmaStep, dmst);
//     m_detectorCurvature->setUseMaxRange(useMaxRange);
//     m_detector = m_detectorCurvature;

//     std::vector<double> phis;
//     std::vector<double> rhos;
//     phis.resize(mvpMapPoints.size());
//     rhos.resize(mvpMapPoints.size());

//     // Transform map point into local frame
//     for (int i = 0; i < mvpMapPoints.size(); i++)
//     {
//         MapPoint *pMP = mvpMapPoints[i];

//         double phi;
//         double rho;
//         MapPoint2LocalPhiRho(pMP, Tcw, phi, rho);

//         phis[i] = phi;
//         rhos[i] = rho;
//     }
//     mpLaserReading = new LaserReading(phis, rhos);
//     m_detector->detect(*mpLaserReading, mvpInterestPoints);
// }

// // Using FLIRT library to describe Interest Points
// void KeyFrame::DescribeInterestPoints()
// {
//     unsigned int scale = 5, dmst = 2, window = 3, detectorType = 0, descriptorType = 0, distanceType = 2, strategy = 0;
//     double baseSigma = 0.2, sigmaStep = 1.4, minPeak = 0.34, minPeakDistance = 0.001, success = 0.95, inlier = 0.4, matchingThreshold = 0.4;
//     bool useMaxRange = false;

//     HistogramDistance<double> *dist = NULL;
//     dist = new Chi2Distance<double>();

//     BetaGridGenerator *m_betaGenerator = NULL;
//     ShapeContextGenerator *m_shapeGenerator = NULL;
//     DescriptorGenerator *m_descriptor = NULL;

//     m_betaGenerator = new BetaGridGenerator(0.02, 0.5, 4, 12);
//     m_betaGenerator->setDistanceFunction(dist);
//     m_descriptor = m_betaGenerator;

//     for (unsigned int j = 0; j < mvpInterestPoints.size(); j++)
//     {
//         mvpInterestPoints[j]->setDescriptor(m_descriptor->describe(*mvpInterestPoints[j], *mpLaserReading));
//     }
// }

// std::vector<InterestPoint *> KeyFrame::GetInterestPoints()
// {
//     return mvpInterestPoints;
// }

void KeyFrame::AddLoopEdge(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mbNotErase = true;
    mspLoopEdges.insert(pKF);
}
void KeyFrame::AddChild(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mspChildrens.insert(pKF);
}

void KeyFrame::ChangeParent(KeyFrame *pKF)
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    mpParent = pKF;
    pKF->AddChild(this);
}

KeyFrame *KeyFrame::GetParent()
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    return mpParent;
}

std::set<KeyFrame *> KeyFrame::GetLoopEdges()
{
    std::unique_lock<std::mutex> lockCon(mMutexConnections);
    return mspLoopEdges;
}

// } // namespace
