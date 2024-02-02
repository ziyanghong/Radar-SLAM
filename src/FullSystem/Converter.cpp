#include"Converter.h"
#define EARTH_RADIUS 6378137.0
#define EDI_RADIUS 6363494.0 // Edinburgh earth radius
#define PI 3.14159265 

// g2o::SE2 to cv::Mat
cv::Mat toCvMat(const g2o::SE2 se2){
    cv::Mat mat = cv::Mat::eye(3, 3, CV_32F);
    Eigen::Vector3d vec = se2.toVector(); 

    mat.at<float>(0, 0) = cos(vec(2));
    mat.at<float>(0, 1) = -sin(vec(2));
    mat.at<float>(1, 0) = sin(vec(2));
    mat.at<float>(1, 1) = cos(vec(2));

    mat.at<float>(0, 2) = vec(0);
    mat.at<float>(1, 2) = vec(1);

    return mat;
}

// Eigen::Vector2d to cv::Mat
cv::Mat toCvMat(const Eigen::Vector2d vec){
    cv::Mat mat = cv::Mat(3, 1, CV_32FC1);
    
    mat.at<float>(0,0) = vec(0);
    mat.at<float>(1,0) = vec(1);
    mat.at<float>(2,0) = 1.0;
    mat.convertTo(mat, CV_32F);

    return mat;
}


// cv::Mat to Eigen Isometry2d
Eigen::Isometry2d cvMat2Eigen(const cv::Mat T_cv )
{
    Eigen::Isometry2d T = Eigen::Isometry2d::Identity();

    for ( int i=0; i<3; i++ )
        for ( int j=0; j<3; j++ ) 
            T(i,j) = T_cv.at<double>(i,j);
  
    return T;
}

// Image points coordinate to local robot frame coordinate
cv::Point2d imageFrame2LocalFrame(int rows, int cols, double scale, cv::Point2d image_point){
    /*
    Image plane:
    0+++++++++++++++x 
    +
    +
    +
    +
    y

    Local robot frame:
            x
            +
            +
            +
            +
    y+++++++0
    */
    // std::cout << "rows: " << rows << std::endl;
    cv::Point2d local_point;
    double half_rows = (rows - 1) / 2;
    double half_cols = (cols - 1) / 2;
    // std::cout << "image_point.y: " << image_point.y << " half_rows: " << half_rows << " scale: " << scale << std::endl;
    double x = (half_rows - image_point.y) / half_rows *scale; // Scale to actual meters 
    double y = (half_cols - image_point.x) / half_cols *scale; // Scale to actual meters
    // std::cout << "x: " << x <<  " y: " << y << std::endl;
    local_point.x = x;
    local_point.y = y;

    return local_point;
}

// local robot frame coordinate to image points coordinate 
cv::Point2d localFrame2ImageFrame(int rows, int cols, double scale, cv::Point2d local_point){
    /*
    Image plane:
    0+++++++++++++++x 
    +
    +
    +
    +
    y

    Local robot frame:
            x
            +
            +
            +
            +
    y+++++++0
    */

    cv::Point2d image_point;

    image_point.x = (cols / 2) - local_point.y / scale * (cols / 2);
    image_point.y = (rows / 2) - local_point.x / scale * (rows / 2);
 
    return image_point;
}

/*
Reference:
https://stackoverflow.com/questions/1185408/converting-from-longitude-latitude-to-cartesian-coordinates
*/
std::vector<double>  gps2Cartesian(double lat, double lon){
    // C++ does not advocate to return the address of a local variable to outside of the function
    // so you would have to define the local variable as static variable.
    std::vector<double> xy;

    double x = EARTH_RADIUS * cos(lat* PI / 180.0 ) * cos(lon* PI / 180.0 );
    xy.push_back(x);
    double y = EARTH_RADIUS * cos(lat* PI / 180.0 ) * sin(lon* PI / 180.0 );
    xy.push_back(y);
    return xy;
}


// XYZRPY to Eigen Transformation matrix
Eigen::Matrix4d xyzrpy2mat(double x, double y, double z, double roll, double pitch, double yaw){

    Eigen::Matrix4d R_X = Eigen::Matrix4d::Identity();
    // R_X << 1,         0,         0,  0,
    //        0, cos(roll), -sin(roll), 0,
    //        0, sin(roll),  cos(roll), 0,
    //        0,         0,          0, 1;  

    Eigen::Matrix4d R_Y = Eigen::Matrix4d::Identity();
    // R_Y <<  cos(pitch), 0, sin(pitch), 0,
    //                  0, 1,          0, 0,
    //        -sin(pitch), 0, cos(pitch), 0,
    //                  0, 0,          0, 1;     

    Eigen::Matrix4d R_Z = Eigen::Matrix4d::Identity();
    R_Z <<  cos(yaw), -sin(yaw), 0, 0,
            sin(yaw),  cos(yaw), 0, 0,
                   0,         0, 1, 0,
                   0,         0, 0, 1;      

    Eigen::Matrix4d RT = R_Z * R_Y * R_X;  
    RT(0,3) = x;
    RT(1,3) = y;
    // RT(2.3) = z;
    return RT;   
}

// Global mappoint to local phi and rho
void MapPoint2LocalPhiRho(MapPoint* pMP, cv::Mat Tcw, double &phi, double &rho){
         // 2D in absolute coordinates
        cv::Mat MapPointWorldPos = pMP->GetWorldPos();
        cv::Mat P = MapPointWorldPos.rowRange(0,2);

        // 2D in local frame coordinate  
        cv::Mat Rcw = Tcw.rowRange(0,2).colRange(0,2);
        cv::Mat Rwc = Rcw.t();
        cv::Mat tcw = Tcw.rowRange(0,2).col(2);
        const cv::Mat Pc = Rwc*P-tcw;   

        phi = atan2(Pc.at<float>(1), Pc.at<float>(0));
        rho = sqrt( pow (Pc.at<float>(0),2) + pow (Pc.at<float>(1), 2) );

}
