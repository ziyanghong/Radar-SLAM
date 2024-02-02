//
//
// FLIRTLib - Fast Laser Interesting Region Transform Library
// Copyright (C) 2009-2010 Gian Diego Tipaldi and Kai O. Arras
//
// This file is part of FLIRTLib.
//
// FLIRTLib is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// FLIRTLib is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with FLIRTLib.  If not, see <http://www.gnu.org/licenses/>.
//

#include "CarmenLog.h"

#include <boost/algorithm/string/predicate.hpp>
#include <math.h>

/** \def MAX_LINE_SIZE The maximum length of a line. */
#define MAX_LINE_SIZE 8192


void detectPeakCFAR(const cv::Mat azimuth_scan, int azimuth_index, std::vector<cv::KeyPoint> &keypoints, int visPointSize,
                    const int num_train, const int num_guard, const double falseAlarmRate)
{

    /* 
    Detecet peaks with Cell Averaging CFAR algorithm

    azimuth_scan; the scan of interest
    azimuth_index; the index of azimuth angle
    keypoints: cv type keypoints
    num_train: Number of training cells.
    num_guard: Number of guard cells.
    falseAlarmRate: False alarm rate.
    */

    int num_cells = azimuth_scan.cols;
    // std::cout << "azimuth_scan cols: " << azimuth_scan.cols << " rows: " << azimuth_scan.rows << std::endl;
    std::cout << "azimuth_index: " << azimuth_index << std::endl;
    int num_train_half = round(num_train / 2);
    int num_guard_half = round(num_guard / 2);
    int num_side = num_train_half + num_guard_half;

    double pw = pow(falseAlarmRate, (-1.0 / (double)num_train));
    //    std::cout<< "pw: " << pw << std::endl;
    double alpha = num_train * (pw - 1.0);
    //    std::cout<< "alpha: " << alpha << std::endl;

    for (int i = 20; i < (num_cells - 40); i++)
    {

        //-- boundaries
        int start = 0;
        int end = 0;
        if ((i - num_side) <= 0)
        {
            start = 0;
            end = num_side * 2;
        }
        else
        {
            start = i - num_side;
            end = i + num_side;
        }

        if ((i + num_side) >= num_cells)
        {
            start = i - num_side * 2;
            end = num_cells;
        }
        else
        {
            start = i - num_side;
            end = i + num_side;
        }
        //-- boundaries

        float sum1 = 0;
        for (int s1 = start; s1 < end; s1++)
        {
            sum1 += azimuth_scan.at<uchar>(0, s1);
        }

        float sum2 = 0;
        for (int s2 = (i - num_guard_half); s2 < (i + num_guard_half); s2++)
        {
            sum2 += azimuth_scan.at<uchar>(0, s2);
        }

        double p_noise = (sum1 - sum2) / num_train;
        double threshold = alpha * p_noise;
        float value = (float)azimuth_scan.at<uchar>(0,i);
 
        if (value > threshold)
        {
            std::cout<< "threshold: " << threshold << std::endl;
            std::cout<< i << "th " <<  "value: " << value << std::endl;                   
            cv::Point2f pt(i, azimuth_index);
            cv::KeyPoint new_keypoint(pt, visPointSize);
            keypoints.push_back(new_keypoint);
        }
    }
}

void detectOnePeak(cv::Mat azimuth_scan, int azimuth_index, std::vector<cv::KeyPoint> &keypoints){
    /*From first to 70th pixle, put them zeros.*/
    for (int i=0; i < 70; i++){
        azimuth_scan.at<uchar>(0,i) = 0;
    }
    /*Points far away, put them zeros.*/
    for (int i=1000; i < azimuth_scan.cols; i++){
        azimuth_scan.at<uchar>(0,i) = 0;
    }

    double minVal;
    double maxVal;
    cv::Point minIdx;
    cv::Point maxIdx;    
    cv::minMaxLoc(azimuth_scan, &minVal, &maxVal, &minIdx, &maxIdx);
    cv::Point2f pt(maxIdx.x, azimuth_index);
    cv::KeyPoint new_keypoint(pt, 50);
    keypoints.push_back(new_keypoint);
}

// Ziyang function
AbstractReading* multipleReadingAssociation(std::vector<AbstractReading *> vMultipleReading)
{
    AbstractReading* reading = NULL;
    if (vMultipleReading.size() == 1){
        reading = vMultipleReading[0];

    }else{
        std::vector<double> phis, rhos;


        const LaserReading* r_init = dynamic_cast<const LaserReading*>(vMultipleReading[0]);
        double x_init = r_init->getLaserPose().x;
        double y_init = r_init->getLaserPose().y;
        double theta_init = r_init->getLaserPose().theta;
        std::cout << "laser pose x_init: " << x_init << ", y_init: " << y_init << ", theta_init: "<< theta_init << std::endl;

        const std::vector<double>& _phi_init = r_init->getPhi();
        const std::vector<double>& _rho_init = r_init->getRho();

        // std::cout << vMultipleReading.size() << std::endl;

        for (int j=0; j< _phi_init.size(); j++){
            phis.push_back(_phi_init[j]);
            rhos.push_back(_rho_init[j]);
        }

        for(int i=1; i<vMultipleReading.size(); i++){
            // std::cout << "i: " << i << std::endl;
            const LaserReading* r = dynamic_cast<const LaserReading*>(vMultipleReading[i]);
            const OrientedPoint2D& laserPose = r->getLaserPose();
            double _x = laserPose.x;
            double _y = laserPose.y;
            double _theta = laserPose.theta;
            const std::vector<double>& _phi = r->getPhi();
            const std::vector<double>& _rho = r->getRho();
            std::cout << "laser pose x: " << _x << ", y: " << _y << ", theta: " << _theta << std::endl;
            // Transform points from other scan reading to the first reading
            for(int j=0; j< _phi.size(); j++){
	            double x = cos(_phi[i] + (_theta - theta_init))*_rho[i] + _x - x_init;
	            double y = sin(_phi[i] + (_theta - theta_init))*_rho[i] + _y - y_init;    
                // std::cout << "From other scans x: " << x << ", y: " << y << std::endl;
                double phi = atan2(y, x);
                double rho = sqrt( pow (x,2) + pow (y, 2) );
                phis.push_back(phi);
                rhos.push_back(rho);
            }

        }
        LaserReading* r_fused = new LaserReading(phis, rhos);
        r_fused->setLaserPose(r_init->getLaserPose());

        reading = r_fused;
    }
    return reading;
}


void CarmenLogReader::readLog(std::istream& _stream, std::vector<AbstractReading*>& _log) const{
    char buffer[MAX_LINE_SIZE];
    int counter = 0;
    std::vector<AbstractReading *> vReading;
    int MAX_FRAME = m_MAX_FRAME; // Ziyang

    while(_stream)
    {
	    _stream.getline(buffer,MAX_LINE_SIZE);
	    std::istringstream instream(buffer);
	    AbstractReading *reading = readLine(instream);
        vReading.push_back(reading);
        std::cout << "counter: " << counter << std::endl;
        counter++;
        if (vReading.size()>=MAX_FRAME && reading){
            AbstractReading *readingMultiple = multipleReadingAssociation(vReading);
	        if (readingMultiple){
	            _log.push_back(readingMultiple);
            }
            std::vector<AbstractReading *>::iterator it; 
            it = vReading.begin(); 
            vReading.erase(it); 
            // vReading.clear();
            // std::cout << std::endl;
        }

    }
}

AbstractReading* CarmenLogReader::readLine(std::istream& _stream) const{
    std::string sensorName;
    _stream >> sensorName;
    _stream.seekg(0, std::ios_base::beg);
    // std::cout << "readline." << std::endl;
    if (boost::iequals(sensorName,"FLASER")){
	return parseFLaser(_stream);
    } else if (boost::istarts_with(sensorName,"ROBOTLASER")){
	return parseRobotLaser(_stream);
    } else if (boost::istarts_with(sensorName,"RAWLASER")){
	return parseRawLaser(_stream);
    } else if (boost::istarts_with(sensorName,"RADAR")){
    return parseRadar(_stream);
    }
    return 0;
}

LaserReading* CarmenLogReader::parseRadar(std::istream& _stream) const{
    // std::cerr << "Parsing radar." << std::endl;
    std::string sensorName, robotName;
    std::vector<double> phi,rho,remission;    
    std::vector<double> xs,ys;
    unsigned int number=0, remissionNumber=0;
    OrientedPoint2D laserPose, robotPose;
    double timestamp;
    double start, fov, resolution, maxRange, accuracy;
    int laserType, remissionMode;
    double tv, rv, forward_safety_dist, side_safety_dist, turn_axis;
    
    _stream >> sensorName >> laserType >> start >> fov >> resolution >> maxRange >> accuracy >> remissionMode; // Laser sensor parameters

    _stream >> number;
    _stream >> remissionNumber;

    std::string radarImagePath;
    _stream >> radarImagePath;

    int rows, cols;
    double scale;
    _stream >> rows >> cols >> scale;

    /*
    //- Detect the keypoints using SURF Detector on cartesian image
    cv::Mat image = cv::imread(radarImagePath, -1);
    int minHessian = 500;
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( minHessian );
    std::vector<cv::KeyPoint> keypoints;
    detector->detect( image, keypoints );
    xs.resize(keypoints.size());
    ys.resize(keypoints.size());
    for (int i=0; i < keypoints.size(); i++){
        xs[i] = (rows / 2 - keypoints[i].pt.y) / (rows / 2) * scale; // Convert to local frame and Scale to actual meters 
        ys[i] = (cols / 2 - keypoints[i].pt.x) / (cols / 2) * scale; // Convert to local frame and Scale to actual meters 
    }
    //std::cout << "Number of keypoints: " << keypoints.size() << std::endl;
    */

    //-- Detect the keypoints using CFAR Detector on polar image
    cv::Mat ori_image = cv::imread(radarImagePath, 0);
    // std::cout << "ori_image rows: " << ori_image.rows << " ori_image cols: " << ori_image.cols << std::endl;
    int maxCOLS = ori_image.cols / (ori_image.cols * accuracy) * maxRange;
    // std::cout << "maxCOLS: " << maxCOLS << std::endl;
    cv::Mat polar_image;
    ori_image(cv::Rect(0, 0, maxCOLS, ori_image.rows)).copyTo(polar_image);
    std::vector<cv::KeyPoint> keypoints_polar;

    /*
    // CFAR Parameters
    int visPointSize = 50;
    int numberOfTrainCell = 40;
    int numberOfGuardCell = 4;
    double probabilityOfFalseAlarm = 0.000005;    
    Extract CFAR keypoints
    for (int a = 0; a < (polar_image.rows - 1); a++)
    {
        detectPeakCFAR(polar_image.row(a), a, keypoints_polar, visPointSize, numberOfTrainCell, numberOfGuardCell, probabilityOfFalseAlarm);
    }
    std::cout <<"CFAR points detected in one frame: " << keypoints_polar.size() << std::endl;
    */

    // Detect the maximum peak 
    for (int a = 0; a < polar_image.rows; a++)
    {   
        // Down sample scans
        if (a % m_DOWN_SAMPLE_SCAN == 0)  
           detectOnePeak(polar_image.row(a), a, keypoints_polar);
    }


    // SURF on polar image
    // int minHessian = 700;
    // cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create( minHessian );
    // detector->detect( polar_image, keypoints_polar ); 


    //-- Draw keypoints
    cv::Mat img_keypoints_1; 
    // cv::drawKeypoints( polar_image, keypoints_polar, img_keypoints_1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );    
    // cv::imshow("Keypoints 1", img_keypoints_1 ); cv::waitKey(0);
    phi.resize(keypoints_polar.size());
    rho.resize(keypoints_polar.size());  
    std::vector<cv::KeyPoint> local_points;  
    for(uint i=0; i < keypoints_polar.size(); i++){
        phi[i] = - static_cast<double>(keypoints_polar[i].pt.y)*resolution;
        rho[i] = keypoints_polar[i].pt.x*accuracy;

        // For opposite direction points descriptor
        if (directionDescriptor == 1){
            rho[i] = -rho[i];
        }
        // std::cout << "keypoints_polar[i].pt.x: " << keypoints_polar[i].pt.x << std::endl;
        // std::cout << "keypoints_polar[i].pt.x*resolution: " << static_cast<double>(keypoints_polar[i].pt.x)*resolution << std::endl;        
        // std::cout << "phi[i]: " << phi[i] << std::endl;        
        // std::cout << "rho[i]: " << rho[i] << std::endl;
        // double x = cos(phi[i])*rho[i] + 87.5;
        // double y = 87.5 - sin(phi[i])*rho[i];
        // cv::Point2f pt(x, y);
        // cv::KeyPoint new_keypoint(pt, 50);
        // local_points.push_back(new_keypoint);        
    }
    // cv::Mat img_local_points;     
    // cv::Mat local_image(176,176, CV_8UC3, cv::Scalar(0,0,0));    
    // cv::drawKeypoints( local_image, local_points, img_local_points, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT );    
    // cv::imshow("local poionts", img_local_points ); cv::waitKey(0);

    _stream >> laserPose.x >> laserPose.y >> laserPose.theta;
    _stream >> robotPose.x >> robotPose.y >> robotPose.theta;
    
    _stream >> tv >> rv >> forward_safety_dist >> side_safety_dist >> turn_axis;
    
    _stream >> timestamp >> robotName;

    LaserReading *result = new LaserReading(phi, rho, timestamp, sensorName, robotName);
    result->setMaxRange(maxRange);
    result->setRemission(remission);
    result->setLaserPose(laserPose);
    result->setRobotPose(robotPose);
    
    return result;    
}

LaserReading* CarmenLogReader::parseRobotLaser(std::istream& _stream) const{
    std::string sensorName, robotName;
    std::vector<double> phi,rho,remission;
    unsigned int number=0, remissionNumber=0;
    OrientedPoint2D laserPose, robotPose;
    double timestamp;
    double start, fov, resolution, maxRange, accuracy;
    int laserType, remissionMode;
    double tv, rv, forward_safety_dist, side_safety_dist, turn_axis;
    
    _stream >> sensorName >> laserType >> start >> fov >> resolution >> maxRange >> accuracy >> remissionMode; // Laser sensor parameters
    
    _stream >> number;
    phi.resize(number);
    rho.resize(number);
    // std::cout << "number of rho: " << number << std::endl;
        std::vector<cv::KeyPoint> local_points;  

    for(uint i=0; i < number; i++){
	phi[i] = start + i*resolution;
	_stream >> rho[i];
    }
  
    

    _stream >> remissionNumber;
    // std::cout << "remissionNumber: " << remissionNumber << std::endl;
    remission.resize(remissionNumber);
    
    for(uint i=0; i<remissionNumber; i++){
	_stream >> remission[i];
    }
    
    _stream >> laserPose.x >> laserPose.y >> laserPose.theta;
    _stream >> robotPose.x >> robotPose.y >> robotPose.theta;
    
    _stream >> tv >> rv >> forward_safety_dist >> side_safety_dist >> turn_axis;
    
    _stream >> timestamp >> robotName;

    LaserReading *result = new LaserReading(phi, rho, timestamp, sensorName, robotName);
    result->setMaxRange(maxRange);
    result->setRemission(remission);
    result->setLaserPose(laserPose);
    result->setRobotPose(robotPose);
    
    // std::cout <<"sensorName: " << sensorName << " laserType: " << laserType << " start: " << start << " fov: " << fov << 
    //                          " resolution: " << resolution << " maxRange: " << maxRange << " accuracy: " << accuracy <<
    //                          " remissionMode: " << remissionMode << " number: " << number << 
    //                          " remissionNumber: " << remissionNumber << std::endl;  
    // std::cout << "timestamp: " << timestamp << std::endl;
    // std::cout << "robotName: " << robotName << std::endl;
                                   
    return result;
}

LaserReading* CarmenLogReader::parseRawLaser(std::istream& _stream) const{
    std::string sensorName, robotName;
    std::vector<double> phi,rho,remission;
    unsigned int number, remissionNumber;
    OrientedPoint2D laserPose, robotPose;
    double timestamp;
    double start, fov, resolution, maxRange, accuracy;
    int laserType, remissionMode;
    
    _stream >> sensorName >> laserType >> start >> fov >> resolution >> maxRange >> accuracy >> remissionMode; // Laser sensor parameters
    
    _stream >> number;
    phi.resize(number);
    rho.resize(number);
    
    for(uint i=0; i < number; i++){
	phi[i] = start + i*resolution;
	_stream >> rho[i];
    }

    _stream >> remissionNumber;
    remission.resize(remissionNumber);
    
    for(uint i=0; i<remissionNumber; i++){
	_stream >> remission[i];
    }
        
    _stream >> timestamp >> robotName;
    
    LaserReading *result = new LaserReading(phi, rho, timestamp, sensorName, robotName);
    result->setMaxRange(maxRange);
    result->setRemission(remission);
//     result->setLaserPose(laserPose);
    
    return result;
}

LaserReading* CarmenLogReader::parseFLaser(std::istream& _stream) const{
    std::string sensorName, robotName;
    std::vector<double> phi,rho;
    unsigned int number;
    OrientedPoint2D laserPose, robotPose;
    double timestamp;
    double start, fov, resolution, maxRange;
    
    _stream >> sensorName >> number;
    
    phi.resize(number);
    rho.resize(number);
    start = -M_PI_2;
    fov = M_PI;
    resolution = fov/number;
    maxRange = 81.9;
    
    for(uint i=0; i<number; i++){
	phi[i] = start + i*resolution;
	_stream >> rho[i];
    }

    _stream >> laserPose.x >> laserPose.y >> laserPose.theta;
    _stream >> robotPose.x >> robotPose.y >> robotPose.theta;
    
    _stream >> timestamp >> robotName;
    
    LaserReading *result = new LaserReading(phi, rho, timestamp, sensorName, robotName);
    result->setMaxRange(maxRange);
    result->setLaserPose(laserPose);
    result->setRobotPose(robotPose);
    
    return result;
}


void CarmenLogWriter::writeLog(std::ostream& _stream, const std::vector<AbstractReading*>& _log) const{
    for(uint i = 0; i < _log.size(); i++){
	writeLine(_stream, _log[i]);
    }
}

void CarmenLogWriter::writeLine(std::ostream& _stream, const AbstractReading* _reading) const{
    if (boost::iequals(_reading->getName(),"FLASER")){
	writeFLaser(_stream, dynamic_cast<const LaserReading*>(_reading));
    } else if (boost::istarts_with(_reading->getName(),"ROBOTLASER")){
	writeRobotLaser(_stream, dynamic_cast<const LaserReading*>(_reading));
    } else if (boost::istarts_with(_reading->getName(),"RAWLASER")){
	writeRawLaser(_stream, dynamic_cast<const LaserReading*>(_reading));
    }
}

void CarmenLogWriter::writeFLaser(std::ostream& _stream, const LaserReading* _reading) const{
    _stream << std::fixed;
    _stream << _reading->getName() << " ";
    
    const std::vector<double>& rho = _reading->getRho();
    _stream << rho.size() << " ";
    
    _stream.precision(3);
    for(uint i = 0; i < rho.size(); i++){
	_stream << rho[i] << " ";
    }
    
    const OrientedPoint2D &laserPose = _reading->getLaserPose();
    const OrientedPoint2D &robotPose = _reading->getRobotPose();
    
    _stream.precision(6);
    _stream << laserPose.x << " " << laserPose.y << " " << laserPose.theta << " ";
    _stream << robotPose.x << " " << robotPose.y << " " << robotPose.theta << " ";
    
    _stream << _reading->getTime() << " " << _reading->getRobot() << " " << _reading->getTime() << std::endl;
}

void CarmenLogWriter::writeRobotLaser(std::ostream& _stream, const LaserReading* _reading) const{
    _stream << std::fixed;
    _stream << _reading->getName() << " " << "0 ";
    
    const std::vector<double>& rho = _reading->getRho();
    const std::vector<double>& phi = _reading->getPhi();

    _stream.precision(6);
    _stream << phi.front() << " " << phi.back() - phi.front() << " " << (phi[1] - phi[0]) << " " << _reading->getMaxRange() << " " << "0.010000 "<< "0 ";
    
    _stream << rho.size() << " ";
    
    _stream.precision(3);
    for(uint i = 0; i < rho.size(); i++){
	_stream << rho[i] << " ";
    }
    
    const std::vector<double>& remission = _reading->getRemission();
    _stream << remission.size() << " ";
    
    for(uint i = 0; i < remission.size(); i++){
	_stream << remission[i] << " ";
    }
    
    const OrientedPoint2D &laserPose = _reading->getLaserPose();
    const OrientedPoint2D &robotPose = _reading->getRobotPose();
    
    _stream.precision(6);
    _stream << laserPose.x << " " << laserPose.y << " " << laserPose.theta << " ";
    _stream << robotPose.x << " " << robotPose.y << " " << robotPose.theta << " ";
    
    _stream << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " " << 0.0 << " ";
    
    _stream << _reading->getTime() << " " << _reading->getRobot() << " " << _reading->getTime() << std::endl;
}

void CarmenLogWriter::writeRawLaser(std::ostream& _stream, const LaserReading* _reading) const{
    _stream << std::fixed;
    _stream << _reading->getName() << " " << "0 ";
    
    const std::vector<double>& rho = _reading->getRho();
    const std::vector<double>& phi = _reading->getPhi();

    _stream.precision(6);
    _stream << phi.front() << " " << phi.back() - phi.front() << " " << (phi[1] - phi[0]) << " " << _reading->getMaxRange() << " " << "0.010000 "<< "0 ";
    
    _stream << rho.size() << " ";
    
    _stream.precision(3);
    for(uint i = 0; i < rho.size(); i++){
	_stream << rho[i] << " ";
    }
    
    const std::vector<double>& remission = _reading->getRemission();
    _stream << remission.size() << " ";
    
    for(uint i = 0; i < remission.size(); i++){
	_stream << remission[i] << " ";
    }
        
    _stream << _reading->getTime() << " " << _reading->getRobot() << " " << _reading->getTime() << std::endl;
}
