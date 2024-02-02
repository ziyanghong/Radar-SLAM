#include "Loader.h"

void LoadImage(const std::string strAssociationFilename, std::vector<std::string> &vstrImageFilenamesRadar, std::vector<double> &vTimestamps)
{

    /*
    txt file line format: Frame: xxxx Time: xxxx.xxxx
    */
    std::cout << "Start loading images" << std::endl;
    std::ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());

    std::string s;
    while (std::getline(fAssociation, s))
    {
        // std::cout << s << '\n';
        if (!s.empty())
        {
            std::string frame, frame_id, time;
            double timestamp;
            std::stringstream ss;
            ss << s; // Frame:
            ss >> frame;

            ss << s; // Frame id
            ss >> frame_id;
            vstrImageFilenamesRadar.push_back(frame_id);

            ss << s; // Time:
            ss >> time;

            ss << s; // Timestamp
            ss >> timestamp;
            vTimestamps.push_back(timestamp);
            // std::cout.precision(20);
            // std::cout << "timestamp: " << timestamp << std::endl;
        }
    }
    std::cout << "Image path loaded........." << std::endl;
}

void LoadGroundtruth(const std::string strGrountruthFile, std::vector<std::vector<double>> &vGroundtruth)
{

    /*
    txt file line format: x y
    */
    // std::cout << "Start loading Groundtruth" << std::endl;
    std::ifstream fAssociation;
    fAssociation.open(strGrountruthFile.c_str());

    std::string s;
    while (std::getline(fAssociation, s))
    {
        std::vector<double> x_y_yaw;
        if (!s.empty())
        {
            double x, y, yaw;
            int frame_id;
            std::stringstream ss;
            // ss << s; // framd id
            // ss >> frame_id;

            ss << s; // x
            ss >> x;

            ss << s; // y
            ss >> y;

            // ss << s; // yaw
            // ss >> yaw;

            x_y_yaw.push_back(x);
            x_y_yaw.push_back(y);
            // x_y_yaw.push_back(yaw);
            vGroundtruth.push_back(x_y_yaw);

            // std::cout.precision(20);
            // std::cout << "timestamp: " << timestamp << std::endl;
        }
    }
    // std::cout << "Groundtruth loaded........." << std::endl;
}

void LoadGPSandIMU(const std::string strAssociationFilename, const std::string strFolderPath, std::vector<double> &vTimestamps,
                   std::vector<std::vector<double>> &vTwist, std::vector<std::vector<double>> &vGPS, std::vector<std::vector<double>> &vIMU)
{

    std::cout << "Start loading gps and imu" << std::endl;

    // Load the association file
    std::ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());

    std::string s;
    while (std::getline(fAssociation, s))
    {
        //   std::cout << s << '\n';
        std::vector<double> lon_lat_alt;
        std::vector<double> twist;
        if (!s.empty())
        {
            std::string frame, frame_id, time;
            double timestamp;
            std::stringstream ss;
            ss << s; // Frame:
            ss >> frame;

            ss << s; // Frame id
            ss >> frame_id;

            // ------------------------------Load Twist, GPS and IMU from each txt file--------------------------
            std::string txt_file = strFolderPath + frame_id + ".txt";
            std::ifstream ftxt;
            ftxt.open(txt_file.c_str());
            std::string s_line;
            std::string substr;
            std::stringstream ss_line;

            // Load Twist
            double v_x, v_y, v_z;
            std::getline(ftxt, s_line);
            // std::cout << "line: " << s_line << std::endl;
            ss_line << s_line;
            getline(ss_line, substr, ',');
            v_x = atof(substr.c_str());
            getline(ss_line, substr, ',');
            v_y = atof(substr.c_str());
            getline(ss_line, substr, ',');
            v_z = atof(substr.c_str());
            // std::cout << "v_x: " << v_x << " v_y: " << v_y << " v_z: " << v_z << std::endl;
            ss_line.clear(); // Clear the stringstream

            double ang_v_x, ang_v_y, ang_v_z;
            std::getline(ftxt, s_line);
            // std::cout << "line: " << s_line << std::endl;
            ss_line << s_line;
            getline(ss_line, substr, ',');
            ang_v_x = atof(substr.c_str());
            getline(ss_line, substr, ',');
            ang_v_y = atof(substr.c_str());
            getline(ss_line, substr, ',');
            ang_v_z = atof(substr.c_str());
            // std::cout << "ang_v_x: " << ang_v_x << " ang_v_y: " << ang_v_y << " ang_v_z: " << ang_v_z << std::endl;
            ss_line.clear(); // Clear the stringstream

            twist.push_back(v_x);
            twist.push_back(v_y);
            twist.push_back(v_z);
            twist.push_back(ang_v_x);
            twist.push_back(ang_v_y);
            twist.push_back(ang_v_z);

            // Load GPS
            std::getline(ftxt, s_line);
            // std::cout << "line: " << s_line << std::endl;
            double lon, lat, alt;
            ss_line << s_line;
            getline(ss_line, substr, ',');
            lon = atof(substr.c_str());
            getline(ss_line, substr, ',');
            lat = atof(substr.c_str());
            getline(ss_line, substr, ',');
            alt = atof(substr.c_str());
            ss_line.clear(); // Clear the stringstream
            std::cout.precision(20);
            // std::cout << "lon: " << lon << " lat: " << lat << " alt: " << alt << std::endl;
            lon_lat_alt.push_back(lon);
            lon_lat_alt.push_back(lat);
            lon_lat_alt.push_back(alt);

            // ------------------------------Load Twist, GPS and IMU from each txt file--------------------------

            ss << s; // Time:
            ss >> time;

            ss << s; // Timestamp
            ss >> timestamp;
            // std::cout << "Timestamp: " << timestamp << std::endl;
            vTimestamps.push_back(timestamp);
        }
        vTwist.push_back(twist);
        vGPS.push_back(lon_lat_alt);
    }

    std::cout << "Receive " << vGPS.size() << " gps positions." << std::endl;
}

ConfigLoader::ConfigLoader()
{
}

ConfigLoader::ConfigLoader(std::string yaml_file)
{
    ReadConfigurations(yaml_file);
}

void ConfigLoader::ReadConfigurations(std::string yaml_file)
{
    cv::FileStorage fs(yaml_file, cv::FileStorage::READ);
    std::cout << "Reading configurations." << std::endl;

    // Get configurations.
    fs["nodeID"] >> nodeID;
    fs["pause"] >> pause;
    fs["loopRate"] >> loopRate;
    fs["descriptorOption"] >> descriptorOption;
    fs["matcherOption"] >> matcherOption;
    fs["localFramesMaxSize"] >> localFramesMaxSize;
    fs["ithFrame"] >> ithFrame;
    fs["keyFrameAngle"] >> keyFrameAngle;
    fs["keyFrameDistance"] >> keyFrameDistance;
    fs["twoFramesOptimization"] >> twoFramesOptimization;

    // Keypoint matching parameters
    fs["LowesRatio"] >> LowesRatio;
    fs["cliqueDistThres"] >> cliqueDistThres;
    fs["treeRadius"] >> treeRadius;

    // System parameters
    // fs["init_pose"] >> init_pose;
    fs["cartImageWidth"] >> cartImageWidth;
    fs["cartImageHeight"] >> cartImageHeight;
    fs["cartImageScale"] >> cartImageScale;

    // Get paths.
    fs["datasetPath"] >> dataset_path;
    fs["polarRadarImageAssociateFile"] >> radar_association_file;
    fs["radarImagePolarFolderPath"] >> radar_image_polar_folder_path;
    fs["radarImageCartesianFolderPath"] >> radar_image_cartesian_folder_path;
    fs["gpsAssociateFile"] >> gps_imu_association_file;
    fs["gpsimuFolderPath"] >> gps_imu_folder_path;
    fs["groundtruthFile"] >> groundtruth_file;
    fs["odometryResultFile"] >> odometry_result_file;
    fs["g2oResult"] >> g2o_result_path;
    fs["logPath"] >> log_path;
    // CFAR parameters
    fs["visPointSize"] >> visPointSize;
    fs["numberOfTrainCell"] >> numberOfTrainCell;
    fs["numberOfGuardCell"] >> numberOfGuardCell;
    fs["probabilityOfFalseAlarm"] >> probabilityOfFalseAlarm;

    // Oxford descriptor parameters
    fs["oxfordRadius"] >> oxfordRadius;
    fs["wedgeResolution"] >> wedgeResolution;
    // SURF parameters
    fs["minHessian"] >> minHessian;

    // Optimizer parameters
    fs["optimizeStep"] >> optimizeStep;
    fs["minMatchesBetweenFrames"] >> minMatchesBetweenFrames;
    fs["landmarkInformation"] >> landmarkInformation;
    fs["poseInformationXY"] >> poseInformationXY;
    fs["poseInformationYaw"] >> poseInformationYaw;
    fs["landmarkMinObservation"] >> landmarkMinObservation;
    fs["startFrameTest"] >> startFrameTest;
    fs["lastFrameTest"] >> lastFrameTest;
    fs["iterations"] >> iterations;
    fs["motionPriorFactorX"] >> motionPriorFactorX;
    fs["motionPriorFactorY"] >> motionPriorFactorY;
    fs["motionPriorFactorYaw"] >> motionPriorFactorYaw;
    fs["loopFactor"] >> loopFactor;
    // fs["convert2LocalFrame"] >> convert2LocalFrame;
    // fs["ceresORg2o"] >> ceresORg2o;
    fs["chiTwo"] >> chiTwo;

    // tracking mode option
    fs["trackMode"] >> trackMode;
    // Loop closure option
    fs["loopClosure"] >> loopClosure;
    // mask center range
    fs["centerMaskRange"] >> centerMaskRange;

    // For Viewer
    fs["pointCloudSize"] >> pointCloudSize;

    // Local window feature tracker parameters
    fs["maximumTrackPoints"] >> maximumTrackPoints;
    minimumPointsANMS = maximumTrackPoints;
    fs["minimumTrackPoints"] >> minimumTrackPoints;
    fs["maximumTrackFrames"] >> maximumTrackFrames;
    numberGridRow = 16;
    numberGridCol = 16;

    // Delete fs.
    fs.release();
    std::cout << "Finished reading paths and parameters." << std::endl
              << std::endl;
}

ConfigLoader::~ConfigLoader()
{
}