#include "FullSystem/System.h"
#include "util/Loader.h"
#include <sys/stat.h> // check path exists
#include <boost/filesystem.hpp>
#include <fstream>

int main(int argc, char **argv)
{

    // Load paths and configurations
    std::string yaml_file = argv[1];
    cv::FileStorage fs(yaml_file, cv::FileStorage::READ);
    ConfigLoader config_loader(yaml_file);
    // Create folder to store the odometry result and copy the yaml file into it.
    std::string parent_directory = config_loader.dataset_path;
    std::string result_directory;
    if (config_loader.loopClosure)
    {
        result_directory = parent_directory + "ro/keyframe/" + std::to_string(config_loader.nodeID) + "/";
    }
    else
    {
        // result_directory = parent_directory + "ro/odometry/" + std::to_string(config_loader.nodeID) + "/";
        result_directory = parent_directory + "ro/consistency/" + std::to_string(config_loader.nodeID) + "/";

    }
    std::cout << "result_directory: " << result_directory << std::endl;
    std::string odom_result_file = result_directory + "our_result_odometry.csv";
    std::cout << "odom result file: " << odom_result_file << std::endl;
    std::string keyframe_pose_file = result_directory + "keyframe_pose.csv";
    std::cout << "keyframe_pose_file: " << keyframe_pose_file << std::endl;

    // int folder_n = 0;
    // std::string result_directory = parent_directory + "ro/" + std::to_string(folder_n) + "/";
    // struct stat buffer;
    // while(stat(result_directory.c_str(), &buffer) == 0){
    //     ++folder_n;
    //     result_directory = parent_directory + "ro/" +  std::to_string(folder_n);
    // }

    config_loader.odometry_result_file = odom_result_file;
    config_loader.keyframe_pose_file = keyframe_pose_file;
    config_loader.g2o_result_path = result_directory + "/";
    // return 0;

    const char *cstr = result_directory.c_str();
    boost::filesystem::path dir(cstr);
    boost::filesystem::create_directory(dir);
    std::ifstream src(yaml_file.c_str(), std::ios::binary);
    std::ofstream dst(result_directory + "/config.yaml", std::ios::binary);
    dst << src.rdbuf();

    // ROS init node
    if (config_loader.nodeID == 0)
    {
        ros::init(argc, argv, "radar_odom");
    }
    else
    {
        std::string node_name = "radar_odom_" + std::to_string(config_loader.nodeID);
        ros::init(argc, argv, node_name);
        std::cerr << "init node: " << node_name << std::endl;
        // std::getchar();
    }
    ros::NodeHandle n;
    ros::Rate r(10); // hz
    // Publish the GPS
    ros::Publisher gps_pub, twist_pub, gt_pub;
    tf::TransformBroadcaster broadcaster;
    gps_pub = n.advertise<visualization_msgs::Marker>("gps", 10);
    twist_pub = n.advertise<visualization_msgs::Marker>("twist_trajectory", 10);
    gt_pub = n.advertise<visualization_msgs::Marker>("gt", 10);

    // Publish radar image
    image_transport::ImageTransport it(n);
    image_transport::Publisher radar_pub = it.advertise("radar_image", 1);

    //--Load image path
    std::string radar_association_file = config_loader.radar_association_file;
    std::string radar_image_polar_folder_path = config_loader.radar_image_polar_folder_path;
    std::string radar_image_cartesian_folder_path = config_loader.radar_image_cartesian_folder_path;
    std::vector<std::string> vstrImageFilenamesRadar;
    std::vector<double> vTimestampsRadar;
    LoadImage(radar_association_file, vstrImageFilenamesRadar, vTimestampsRadar);
    std::cout << "The total number of images is " << vstrImageFilenamesRadar.size() << std::endl;
    //--Load GPS and IMU
    std::string gps_imu_association_file = config_loader.gps_imu_association_file;
    std::string gps_imu_folder_path = config_loader.gps_imu_folder_path;
    std::vector<std::vector<double>> vTwist, vGPS, vIMU;
    std::vector<double> vTimestampsGPS;
    // LoadGPSandIMU(gps_imu_association_file, gps_imu_folder_path, vTimestampsGPS, vTwist, vGPS, vIMU);
    //--Load groundtruth
    std::string groundtruth_file = config_loader.groundtruth_file;
    std::vector<std::vector<double>> vGroundtruth;
    LoadGroundtruth(groundtruth_file, vGroundtruth);
    visualization_msgs::Marker line_strip;
    line_strip.scale.x = 1;
    line_strip.ns = "gt";
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.header.frame_id = "/world";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;
    std::cout << "size of groundtruth: " << vGroundtruth.size() << std::endl;

    // Initialized the whole system with config loader
    System system(config_loader);


    // // Send TF
    // tf::Transform transform;
    // transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    // tf::Quaternion q;
    // q.setRPY(0.0, 0.0, 0.0);
    // transform.setRotation(q);
    // broadcaster.sendTransform(
    //     tf::StampedTransform(transform,
    //                          ros::Time::now(), "world", "base_link"));
    // // publish
    // twist_pub.publish(twist_marker);


    auto start = std::chrono::high_resolution_clock::now();

    int frameIdx = 1;
    g2o::SE2 posePrev(0.0, 0.0, 0.0);

    // Main loop
    for (std::size_t i = 0; i < vstrImageFilenamesRadar.size() - 1; i++)
    // for (std::size_t i = config_loader.startFrameTest; i < config_loader.lastFrameTest; i++)
    // for (std::size_t i = 0; i < 300; i++)
    {
        std::vector<double> gt = vGroundtruth[i];

        geometry_msgs::Point p;
        p.x = gt[0];
        p.y = gt[1];
        p.z = 0.0;
        std::cout << "x: " << p.x << " y: " << p.y << std::endl;
        g2o::SE2 poseCurr(gt[0], gt[1], gt[2]);

        line_strip.points.push_back(p);
        gt_pub.publish(line_strip);

        // Load image
        std::string polar_image_path = radar_image_polar_folder_path + vstrImageFilenamesRadar[i] + ".png";
        std::cout << "polar_image_path: " << polar_image_path << std::endl;
        cv::Mat polar_image = cv::imread(polar_image_path, -1);
        if (!polar_image.data) // Check for invalid input
        {
            std::cout << "Could not open or find the polar image" << std::endl;
            return -1;
        }

        std::string cart_image_path = radar_image_cartesian_folder_path + vstrImageFilenamesRadar[i] + ".png";
        std::cout << "cart_image_path: " << cart_image_path << std::endl;
        cv::Mat cart_image = cv::imread(cart_image_path, -1);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", cart_image).toImageMsg();
        radar_pub.publish(msg);
        if (!cart_image.data) // Check for invalid input
        {   
            std::cout << "Could not open or find the cartesian image" << std::endl;
            return -1;
        }
        // // Compute image intensity mean and subtract it to account for inter-frame intensity changes
        // cv::Scalar mean = cv::mean(cart_image);
        // cv::subtract(cart_image, mean, cart_image);

        // g2o::SE2 relativePose = posePrev.inverse() * poseCurr ;
        // std::cout << "Relative Pose between current frame and last frame: " << std::endl
        //           << relativePose.toVector() << std::endl;

        posePrev = poseCurr;

        // Track
        system.mpTracker->GrabSensorInput(frameIdx, vTimestampsRadar[i], polar_image, cart_image);
        std::cout << std::endl;

        frameIdx++;
        // Sleep
        // r.sleep();
    }
    std::cout << "Finished one sequence: " << std::to_string(config_loader.nodeID) << std::endl;
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop - start);
    std::cout << "Finished one sequence using " << duration.count() << " seconds." << std::endl;


    std::cout << "Saving map points..." << std::endl;
     system.SaveAllFeaturePoints();
    std::cout << "Saving keyframe poses..." << std::endl;
    system.SaveKeyFramePoses();

    return 0;
}