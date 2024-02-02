#include "Frame.h"




// namespace ROAM{

Frame::Frame()
{

}

Frame::~Frame()
{
    // std::cout << "Deleted frame with mnId: "<< mnId << std::endl; 
}

Frame::Frame(std::size_t frame_index, double timestampCurrent, double timestampPrevious,
            const cv::Mat &PolarImage, const cv::Mat &CartImage, ConfigLoader _config_loader):mnId(frame_index), 
            mTimestamp(timestampCurrent), mPreviousTimestamp(timestampPrevious),
            mbReady(false), mPolarImage(PolarImage), mCartImage(CartImage), config_loader(_config_loader),
            rows(_config_loader.cartImageHeight), cols(_config_loader.cartImageWidth), scale(_config_loader.cartImageScale)
{
    mbM2dpReady = false;

    if (_config_loader.trackMode==0)
    {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        ExtractKeyPoints(frame_index, PolarImage, CartImage, _config_loader, keypoints, descriptors);

        mvKeypoints = keypoints;
        mvDescriptors = descriptors.clone();

        N = mvKeypoints.size();
        mvpMapPoints = std::vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
        mvbOutlier = std::vector<bool>(N,false);
    }

    // if (_config_loader.loopClosure){
    
    //     // Create a new thread to compute M2DP global descriptor
    //     mbM2dpReady = false;
    //     std::thread new_thread(&Frame::ComputeGlobalDescriptor, this);
    // }else
    // {
    //     mbM2dpReady = true;
    // }

}

/*
    * M2DP global descriptor extraxtion */
void Frame::ComputeGlobalDescriptor()
{

    // auto start = std::chrono::high_resolution_clock::now();

    cv::Mat PolarImage;
    double max_selected_distance = config_loader.cartImageScale;
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

    mbM2dpReady = true;

    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // std::cout << "Frame::M2DP computation time is: " << duration.count() << " milliseconds." << std::endl;
}


void Frame::SetKeypoints(const std::vector<cv::KeyPoint> &keypoints)
{
    mvKeypoints = keypoints;
    N = mvKeypoints.size();
    mvpMapPoints = std::vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = std::vector<bool>(N,false);    
}

void Frame::SetDescriptors(const cv::Mat &descriptors)
{
    mvDescriptors = descriptors.clone();
}


bool Frame::IsReady()
{
    return mbReady;
}

bool Frame::IsM2dpReady()
{
    return mbM2dpReady;
}

void Frame::ExtractKeyPoints(const std::size_t& index, const cv::Mat &PolarImage, const cv::Mat CartImage, 
                    const ConfigLoader config_loader, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{   
    cv::Mat polar_image = PolarImage;
    cv::Mat cart_image = CartImage;
    cv::Mat img_keypoints;

    // Mask for keypoints detection
    cv::Mat mask = cv::Mat(cart_image.size(),CV_8UC1);
    mask = 255 ;

    // mask out the region plus 2 and minus 2 meter from the center
    int meters = config_loader.centerMaskRange;
    int offset = round(cols/2.0 /scale)*meters;
    // std::cout << "pixel offset: " << offset << std::endl;
    mask.colRange(round(cols/2.0) - offset, round(cols/2.0) + offset) = 0; 

    // std::cout << "descriptorOption: " << config_loader.descriptorOption << std::endl;
    switch (config_loader.descriptorOption)
    {
    case 1:
    {
        printf("CFAR!");

        // CFAR Parameters
        int visPointSize = config_loader.visPointSize;
        int numberOfTrainCell = config_loader.numberOfTrainCell;
        int numberOfGuardCell = config_loader.numberOfGuardCell;
        double probabilityOfFalseAlarm = config_loader.probabilityOfFalseAlarm;

        auto start = std::chrono::high_resolution_clock::now();
        std::vector<cv::KeyPoint> keypoints_polar;
        // Extract CFAR keypoints
        for (int a = 0; a < (polar_image.cols - 1); a++)
        {
            detectPeakCFAR(polar_image.col(a), a, keypoints_polar, visPointSize, numberOfTrainCell, numberOfGuardCell, probabilityOfFalseAlarm);
        }

        std::cout << "Number of keypoints detected: " << keypoints_polar.size() << " in one polar_image." << std::endl;
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "CFAR computation time is: " << duration.count() << " milliseconds." << std::endl;

        // Remap the keypoints to Cartesian coordinate
        std::vector<cv::KeyPoint> KeyPointsCartesian;
        polar_to_cartesian(polar_image, cart_image, keypoints_polar, KeyPointsCartesian);

        // Compute descriptors using the Oxford paper
        // computeDescriptor(cart_image, config_loader.oxfordRadius, config_loader.wedgeResolution, KeyPointsCartesian, descriptors);

        // Compute descriptors using SURF
        extractSurfDescriptor(config_loader, cart_image, KeyPointsCartesian, descriptors);
        keypoints = KeyPointsCartesian;

        //-- Draw polar keypoints
        cv::drawKeypoints(polar_image, keypoints_polar, img_keypoints);
        cv::namedWindow("Display original image", cv::WINDOW_AUTOSIZE); // Create a window for display.
        cv::imshow("Keypoints Polar", img_keypoints);
        //-- Draw Cartesian keypoints
        cv::drawKeypoints(cart_image, KeyPointsCartesian, img_keypoints);
        // cv::namedWindow("Display cartesian image", cv::WINDOW_AUTOSIZE); // Create a window for display.
        cv::resize(img_keypoints, img_keypoints, cv::Size(800, 800), cv::INTER_NEAREST);
        // cv::putText(img_keypoints,
        //             "CFAR keypoints",
        //             cv::Point(5, 20),               // Coordinates
        //             cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
        //             1.0,                            // Scale. 2.0 = 2x bigger
        //             cv::Scalar(255, 255, 255),      // BGR Color
        //             1);                             // Line Thickness (Optional)
        // cv::imshow("CFAR Keypoints Cartesian", img_keypoints);
        // cv::waitKey(0);

        break;
    }
    case 2:
    {
        printf("SURF!\n");
        auto start = std::chrono::high_resolution_clock::now();

        // SURF
        int minHessian = config_loader.minHessian;
        int nOctaves = 4;
        int nOctaveLayers = 3;
        bool extended = false;
        bool upright = true;    
        cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian, nOctaves, nOctaveLayers, extended, upright);
        detector->detectAndCompute(cart_image, mask, keypoints, descriptors);

        // // SIFT
        // cv::Ptr<cv::Feature2D> f2d = cv::xfeatures2d::SIFT::create();
        // f2d->detect(cart_image, keypoints);
        // f2d->compute( cart_image, keypoints, descriptors );

        // // ORB
        // cv::Ptr<cv::ORB> detector = cv::ORB::create();
        // detector->detectAndCompute(cart_image, cv::noArray(), keypoints, descriptors);
        
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "Keypoint descriptors computation time is: " << duration.count() << " milliseconds." << std::endl;
        // //-- Draw keypoints
        cv::drawKeypoints(cart_image, keypoints, img_keypoints);
        // cv::putText(img_keypoints,
        //             "SURF keypoints",
        //             cv::Point(5, 20),               // Coordinates
        //             cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
        //             1.0,                            // Scale. 2.0 = 2x bigger
        //             cv::Scalar(255, 255, 255),      // BGR Color
        //             1);                             // Line Thickness (Optional)
        // cv::namedWindow("Display cartesian image", cv::WINDOW_AUTOSIZE); // Create a window for display.
        // cv::imshow("Keypoints Cartesian", img_keypoints);
        // cv::waitKey(0);

        // resize it 
        cv::resize(img_keypoints, img_keypoints, cv::Size(800, 800), cv::INTER_NEAREST);

        break;
    }
    default:
        printf("What the heck is that!");
        break;
    }

    
    // KD-tree of keypoints in local frame
    pointVec points;
    point_t pt;
    for (int i=0; i<keypoints.size();i++){
        cv::Point2d local_point = imageFrame2LocalFrame(rows, cols, scale, keypoints[i].pt);
        pt = {local_point.x, local_point.y};
        points.push_back(pt);
    }

    std::cout  << "cols: " << cols << std::endl;
    KDTree _Tree(points);
    mTree = _Tree;

    mLocalPoints = points;
    mKeyPointsImage = img_keypoints;
    // Convert to CV_32F
    descriptors.convertTo(descriptors, CV_32F);



    printf("Exit from ExtractKeyPoints\n");
}

bool Frame::IsVisibleInRadius(MapPoint* pMP, double radiusInMeters, double noise){

    // 2D in absolute coordinates
    cv::Mat MapPointWorldPos = pMP->GetWorldPos();
    // std::cout << "MapPoint WorldPos:\n" << MapPointWorldPos << std::endl;

    cv::Mat P = MapPointWorldPos.rowRange(0,2);
    // std::cout << "P:\n" << P << std::endl;

    double distance = sqrt( pow( (TcwSVD.at<float>(0, 2) - MapPointWorldPos.at<float>(0,0)) , 2) +
                            pow( (TcwSVD.at<float>(1, 2) - MapPointWorldPos.at<float>(1,0)) , 2) );
    
    if (distance > (radiusInMeters + noise))
        return false;

    // 2D in local frame coordinate  
    cv::Mat Rcw = TcwSVD.rowRange(0,2).colRange(0,2);
    cv::Mat Rwc = Rcw.t();
    // std::cout << "Rwc:\n" << Rwc << std::endl;
    cv::Mat tcw = TcwSVD.rowRange(0,2).col(2);
    // std::cout << "tcw:\n" << tcw << std::endl;
    const cv::Mat Pc = Rwc* P - tcw;
    // std::cout << "Pc:\n" << Pc << std::endl;

    // Data used by the tracking
    pMP->mbTrackInView = true;

    // cv::Point2d local_point(Pc.at<float>(0), Pc.at<float>(1));
    // cv::Point2d image_point = localFrame2ImageFrame(rows, cols, scale, local_point);    

    // pMP->mTrackProjX = image_point.x;
    // pMP->mTrackProjXR =(radiusInMeters + noise) / scale * (rows / 2);
    // pMP->mTrackProjY = image_point.y;
    // std::cout<<"pMP->mTrackProjX: " << image_point.x << " pMP->mTrackProjY: " << image_point.y << std::endl;

    return true;

}

void Frame::SetPose(const cv::Mat &Tcw_)
{
    Tcw_.copyTo(Tcw);
}

void Frame::SetRT(const cv::Mat &RT_)
{
    RT_.copyTo(RT2KeyFrame);
}

void Frame::SetReferenceKeyFrame(KeyFrame* pReferenceKeyFrame)
{
    mpReferenceKeyFrame = pReferenceKeyFrame;
}

KeyFrame* Frame::GetReferenceKeyFrame()
{
    return mpReferenceKeyFrame;
}

void Frame::SetVelocity(const Eigen::Matrix<double, 3, 1> v)
{
    mVelocity = v;
}

Eigen::Matrix<double, 3, 1> Frame::GetVelocity()
{
    return mVelocity;
}


cv::Mat Frame::GetPose()
{
    return Tcw.clone();
}

cv::Mat Frame::GetFeature()
{
    return mFeature.clone();
}

cv::Mat Frame::GetPointcloud()
{
    return mvPointCloud.clone();
}


cv::Mat Frame::GetPoseInverse()
{
    // std::cout << "GetPoseInverse Tcw: " << std::endl << Tcw << std::endl;
    cv::Mat Rcw = Tcw.rowRange(0,2).colRange(0,2);
    cv::Mat tcw = Tcw.rowRange(0,2).col(2);
    cv::Mat Rwc = Rcw.t();
    cv::Mat Ow = -Rwc*tcw;

    Twc = cv::Mat::eye(3,3,Tcw.type());
    Rwc.copyTo(Twc.rowRange(0,2).colRange(0,2));
    Ow.copyTo(Twc.rowRange(0,2).col(2));      
    return Twc.clone();
}

cv::Mat Frame::GetRT()
{
    return RT2KeyFrame.clone();
}

// } // namespace ROAM