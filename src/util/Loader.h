#ifndef LOADER_H
#define LOADER_H

// Standard library
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <math.h>
#include <iterator>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

// For yaml
#include "opencv2/core.hpp"
#include "opencv2/core/eigen.hpp"

void LoadGroundtruth(const std::string strGrountruthFile, std::vector<std::vector<double>> &vGroundtruth);
void LoadImage(const std::string strAssociationFilename, std::vector<std::string> &vstrImageFilenamesRadar, std::vector<double> &vTimestamps);
void LoadGPSandIMU(const std::string strAssociationFilename, const std::string strFolderPath, std::vector<double> &vTimestamps,
std::vector<std::vector<double>> &vTwist, std::vector<std::vector<double>> &vGPS, std::vector<std::vector<double>> &vIMU);

class ConfigLoader {
public:

	// -----------------------------Configurations-----------------------
	double loopRate;
	int pause;
	int descriptorOption;
	int matcherOption;
	int localFramesMaxSize;
	int ithFrame;
	float keyFrameAngle;
	float keyFrameDistance;
	int twoFramesOptimization;
	int nodeID;
	// ------------------------------------------------------------------

	// ------------------------Keypoint matching-------------------------
	float cliqueDistThres;
	int treeRadius;
	float LowesRatio;
	// ------------------------------------------------------------------

	// ---------------------------System parameters----------------------
    // cv::Mat init_pose = cv::Mat::eye(3, 3, CV_32F);
	int cartImageWidth;
	int cartImageHeight;
	double cartImageScale;
	// ------------------------------------------------------------------

	// ---------------------------Tracking mode--------------------------
	int trackMode;
	// ------------------------------------------------------------------


	// ---------------------------------Paths----------------------------
	std::string dataset_path;
	std::string gps_imu_association_file;
	std::string gps_imu_folder_path;
	std::string groundtruth_file;
	std::string radar_association_file;
	std::string radar_image_polar_folder_path;
	std::string radar_image_cartesian_folder_path;
	std::string odometry_result_file;
	std::string keyframe_pose_file;
	std::string g2o_result_path;
	std::string log_path;
	// ------------------------------------------------------------------

	// ---------------------------CFAR parameters------------------------
	int visPointSize;
	int numberOfTrainCell;
	int numberOfGuardCell;
	double probabilityOfFalseAlarm;
	// ------------------------------------------------------------------

	// ------------------------------Oxford------------------------------
	int oxfordRadius;
	int wedgeResolution;
	// ------------------------------------------------------------------

	// ---------------------------SURF parameters------------------------
	int minHessian;
	// ------------------------------------------------------------------

	// ----------------------Optimizer parameters------------------------
	int minMatchesBetweenFrames;
	int startFrameTest;
	int lastFrameTest;
	int landmarkMinObservation;
	int optimizeStep;
	int iterations;
	
	double huberWidth; //  Robust Kernel
	double landmarkInformation;
	double poseInformationXY;
	double poseInformationYaw;
	double chiTwo;
	double motionPriorFactorX;
	double motionPriorFactorY;
	double motionPriorFactorYaw;
	double loopFactor;
	
	bool convert2LocalFrame;
	bool ceresORg2o;
	// ------------------------------------------------------------------


	// ----------------local feature tracker parameters------------------
    int maximumTrackPoints;
    int minimumTrackPoints;
    int maximumTrackFrames;
	int minimumPointsANMS;
	int numberGridRow;
	int numberGridCol;
	// ------------------------------------------------------------------



	// Viewer
	int pointCloudSize;

	// Loop closure option
	bool loopClosure;
	// Mask center area range in meters
	int centerMaskRange;


	// ConfigLoader
	ConfigLoader();
	ConfigLoader(std::string);
	void ReadConfigurations(std::string);
	~ConfigLoader();

};
#endif