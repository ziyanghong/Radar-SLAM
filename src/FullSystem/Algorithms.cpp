#include "Algorithms.h"
#include "mcqd.h"
#include <numeric>
#include "anms.h"

cv::Mat updateCovariance(cv::Mat Pk_prev, double *Xk, double *Uk, double *processNoises)
{
    cv::Mat Ak = cv::Mat::eye(3, 3, CV_32F);
    Ak.at<float>(0, 2) = -sin(Xk[2]) * Uk[0] - cos(Xk[2]) * Uk[1];
    Ak.at<float>(1, 2) = cos(Xk[2]) * Uk[0] - sin(Xk[2]) * Uk[1];
    // std::cout << "Ak: " << std::endl << Ak << std::endl;

    // Compute Qk base on the odometry by knowing the drift
    cv::Mat Qk = cv::Mat::eye(3, 3, CV_32F);

    Qk.at<float>(0, 0) = processNoises[0] * fabs(Uk[0]) * processNoises[0] * fabs(Uk[0]);
    Qk.at<float>(1, 1) = processNoises[1] * fabs(Uk[1]) * processNoises[1] * fabs(Uk[1]);
    Qk.at<float>(2, 2) = processNoises[2] * fabs(Uk[2]) * processNoises[2] * fabs(Uk[2]);
    // std::cout << "Qk: " << std::endl << Qk << std::endl;

    cv::Mat Wk = cv::Mat::eye(3, 3, CV_32F);
    Wk.at<float>(0, 0) = cos(Xk[2]);
    Wk.at<float>(0, 1) = -sin(Xk[2]);
    Wk.at<float>(0, 2) = 0.0;
    Wk.at<float>(1, 0) = sin(Xk[2]);
    Wk.at<float>(1, 1) = cos(Xk[2]);
    Wk.at<float>(1, 2) = 0.0;
    Wk.at<float>(2, 0) = 0.0;
    Wk.at<float>(2, 1) = 0.0;
    Wk.at<float>(2, 2) = 1.0;

    // std::cout << "Wk: " << std::endl << Wk << std::endl;

    cv::Mat P1 = Ak * Pk_prev * Ak.t();

    // std::cout << "P1: " << std::endl << P1 << std::endl;

    cv::Mat P2 = Wk * Qk * Wk.t();
    // std::cout << "P2: " << std::endl << P2 << std::endl;

    cv::Mat Pk = P1 + P2;
    // std::cout << "Pk:" << std::endl << Pk << std::endl;
    return Pk;
}

// Sorting algorithms for ordering the keyframe index
void sortrows(std::vector<std::vector<double>> &matrix, int col)
{
    std::sort(matrix.begin(),
              matrix.end(),
              [col](const std::vector<double> &lhs, const std::vector<double> &rhs) {
                  return lhs[col] > rhs[col];
              });
}

cv::Mat estimateMotionSVD(const ConfigLoader &config_loader, std::vector<cv::Point2f> prev, std::vector<cv::Point2f> curr, bool &trackOK)
{
    /* 
    Reference1: https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
    Reference2: https://gist.github.com/JiaxiangZheng/8168862
    */

    cv::Mat cv_rt = cv::Mat::eye(3, 3, CV_32F);
    assert(prev.size() == curr.size());

    // SVD
    Eigen::Vector2d center_src(0, 0), center_dst(0, 0);
    int num_pts = curr.size();
    for (int i = 0; i < num_pts; i++)
    {
        cv::Point2d pt_curr = curr[i];
        // std::cout << "curr[i]: " << curr[i] << std::endl;
        if (config_loader.convert2LocalFrame)
        {
            pt_curr = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth, config_loader.cartImageScale, pt_curr);
        }
        // std::cout << "pt_curr.x: " << pt_curr.x << std::endl;
        // std::cout << "pt_curr.y: " << pt_curr.y << std::endl;

        center_src[0] += pt_curr.x;
        center_src[1] += pt_curr.y;
        // std::cout << "center_src: " << center_src << std::endl;

        cv::Point2d pt_prev = prev[i];
        if (config_loader.convert2LocalFrame)
        {
            pt_prev = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth, config_loader.cartImageScale, pt_prev);
        }
        center_dst[0] += pt_prev.x;
        center_dst[1] += pt_prev.y;
        // std::cout << "center_dst: " << center_dst << std::endl;
    }
    // std::cout << "num_pts: " << num_pts << std::endl;
    center_src /= (double)num_pts;
    center_dst /= (double)num_pts;
    // std::cout << "estimateMotionSVD:: center_dst: " << center_dst << std::endl <<  " center_src: " << center_src << std::endl;

    Eigen::MatrixXd S(num_pts, 2), D(num_pts, 2);
    for (int i = 0; i < num_pts; i++)
    {
        cv::Point2d pt_curr = curr[i];
        if (config_loader.convert2LocalFrame)
        {
            pt_curr = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth, config_loader.cartImageScale, pt_curr);
        }
        S(i, 0) = pt_curr.x - center_src[0];
        S(i, 1) = pt_curr.y - center_src[1];

        cv::Point2d pt_prev = prev[i];
        if (config_loader.convert2LocalFrame)
        {
            pt_prev = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth, config_loader.cartImageScale, pt_prev);
        }
        D(i, 0) = pt_prev.x - center_dst[0];
        D(i, 1) = pt_prev.y - center_dst[1];
    }
    Eigen::MatrixXd Dt = D.transpose();
    Eigen::Matrix2d H = Dt * S;
    Eigen::Matrix2d W, U, V;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    Eigen::MatrixXd H_(2, 2);
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j)
            H_(i, j) = H(i, j);
    svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV);
    if (!svd.computeU() || !svd.computeV())
    {
        std::cout << "decomposition error in SVD" << std::endl;
        trackOK = false;
        return cv_rt;
    }
    Eigen::Matrix2d Vt = svd.matrixV().transpose();
    Eigen::Matrix2d R = svd.matrixU() * Vt;
    Eigen::Vector2d t = center_dst - R * center_src;

    Eigen::Matrix3d RT = Eigen::Matrix3d::Identity(3, 3);
    RT.block<2, 2>(0, 0) = R;
    RT(0, 2) = t(0);
    RT(1, 2) = t(1);
    trackOK = true;

    cv::eigen2cv(RT, cv_rt);
    return cv_rt;
}

bool next_iteration = false;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}

cv::Mat estimateMotionICP(const ConfigLoader config_loader, std::vector<cv::Point2f> prev, std::vector<cv::Point2f> curr, bool &trackOK)
{
    // Set verbose level
    // pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    auto start = std::chrono::high_resolution_clock::now();
    int iterations = 20;
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    cv::Mat cv_rt = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat cv_transform = cv::Mat::eye(4, 4, CV_32F);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_dst->points.resize(prev.size());
    for (size_t i = 0; i < cloud_dst->points.size(); ++i)
    {
        cv::Point2d pt_prev = prev[i];
        pt_prev = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth, config_loader.cartImageScale, pt_prev);
        cloud_dst->points[i].x = pt_prev.x;
        cloud_dst->points[i].y = pt_prev.y;
        cloud_dst->points[i].z = 0.0;
    }

    cloud_src->points.resize(curr.size());
    for (size_t i = 0; i < cloud_src->points.size(); ++i)
    {
        cv::Point2d pt_curr = curr[i];
        pt_curr = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth, config_loader.cartImageScale, pt_curr);
        cloud_src->points[i].x = pt_curr.x;
        cloud_src->points[i].y = pt_curr.y;
        cloud_src->points[i].z = 0.0;
    }
    // std::cout << "Saved " << cloud_dst->points.size () << " data points to input:" << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_dst);
    // icp.setMaxCorrespondenceDistance(5);
    icp.setRANSACIterations(iterations);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    trackOK = icp.hasConverged();
    Eigen::Matrix4f rt = icp.getFinalTransformation();
    cv::eigen2cv(rt, cv_transform);

    // Retrieve to cvMat
    cv_rt.at<float>(0, 0) = cv_transform.at<float>(0, 0);
    cv_rt.at<float>(1, 0) = cv_transform.at<float>(1, 0);
    cv_rt.at<float>(0, 1) = cv_transform.at<float>(0, 1);
    cv_rt.at<float>(1, 1) = cv_transform.at<float>(1, 1);
    cv_rt.at<float>(0, 2) = cv_transform.at<float>(0, 3);
    cv_rt.at<float>(1, 2) = cv_transform.at<float>(1, 3);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    // if (icp.hasConverged())
    // {
    //     std::cout << "ICP RANSAC has converged"
    //               << " in " << duration.count() << " ms." << std::endl;
    // }
    // else
    // {
    //     std::cout << "ICP RANSAC has not converged." << std::endl;
    // }

    return cv_rt;
}

cv::Mat estimatePointCloudMotionICP(const ConfigLoader config_loader,
                                    cv::Mat &prev,
                                    cv::Mat &curr,
                                    bool &trackOK)
{

    auto start = std::chrono::high_resolution_clock::now();
    const int iterations = 5;
    const int skip = 1;
    const double angle_diff_threshold = 1.57; // around 45 degree
    cv::Mat cv_rt = cv::Mat::eye(3, 3, CV_32F);
    cv::Mat cv_transform = cv::Mat::eye(4, 4, CV_32F);

    //Perform PCA analysis
    cv::PCA pca_prev(prev, cv::Mat(), cv::PCA::DATA_AS_ROW);
    cv::PCA pca_curr(curr, cv::Mat(), cv::PCA::DATA_AS_ROW);
    cv::Point2d first_vector_prev = cv::Point2d(pca_prev.eigenvectors.at<double>(0, 0), pca_prev.eigenvectors.at<double>(0, 1));
    cv::Point2d first_vector_curr = cv::Point2d(pca_curr.eigenvectors.at<double>(0, 0), pca_curr.eigenvectors.at<double>(0, 1));
    double angle_vec_prev = atan2(first_vector_prev.y, first_vector_prev.x);
    double angle_vec_curr = atan2(first_vector_curr.y, first_vector_curr.x);
    double angle_diff = angle_vec_prev - angle_vec_curr;

    if (angle_diff > angle_diff_threshold)
    {
        trackOK = false;
        return cv_rt;
    }

    cv::Mat RotationMat = cv::Mat::zeros(2, 2, CV_64F);
    RotationMat.at<double>(0, 0) = cos(angle_diff);
    RotationMat.at<double>(0, 1) = -sin(angle_diff);
    RotationMat.at<double>(1, 0) = sin(angle_diff);
    RotationMat.at<double>(1, 1) = cos(angle_diff);

    cv::Mat preAlignT = cv::Mat::eye(3, 3, CV_32F);
    preAlignT.at<float>(0, 0) = cos(angle_diff);
    preAlignT.at<float>(0, 1) = -sin(angle_diff);
    preAlignT.at<float>(1, 0) = sin(angle_diff);
    preAlignT.at<float>(1, 1) = cos(angle_diff);
    // Pre-align point cloud so that the the rotation is minimized
    cv::Mat rotatedPoints = RotationMat * curr.t();

    // ICP
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_dst->points.resize(prev.rows);
    for (size_t i = 0; i < cloud_dst->points.size(); ++i)
    {
        if (i % skip == 0)
        {
            cloud_dst->points[i].x = prev.at<double>(i, 0);
            cloud_dst->points[i].y = prev.at<double>(i, 1);
            cloud_dst->points[i].z = 0.0;
        }
    }

    cloud_src->points.resize(curr.rows);
    for (size_t i = 0; i < cloud_src->points.size(); ++i)
    {
        if (i % skip == 0)
        {
            cloud_src->points[i].x = rotatedPoints.at<double>(0, i);
            cloud_src->points[i].y = rotatedPoints.at<double>(1, i);
            cloud_src->points[i].z = 0.0;
        }
    }
    // std::cout << "Saved " << cloud_dst->points.size () << " data points to input:" << std::endl;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_src);
    icp.setInputTarget(cloud_dst);
    // icp.setMaxCorrespondenceDistance(20.0);
    icp.setRANSACIterations(iterations);

    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    trackOK = icp.hasConverged();
    Eigen::Matrix4f rt = icp.getFinalTransformation();
    cv::eigen2cv(rt, cv_transform);

    // Retrieve to cvMat
    cv_rt.at<float>(0, 0) = cv_transform.at<float>(0, 0);
    cv_rt.at<float>(1, 0) = cv_transform.at<float>(1, 0);
    cv_rt.at<float>(0, 1) = cv_transform.at<float>(0, 1);
    cv_rt.at<float>(1, 1) = cv_transform.at<float>(1, 1);
    cv_rt.at<float>(0, 2) = cv_transform.at<float>(0, 3);
    cv_rt.at<float>(1, 2) = cv_transform.at<float>(1, 3);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    if (icp.hasConverged() == 1)
    {
        // std::cout << "ICP RANSAC has converged"
        //           << " in " << duration.count() << " ms." << std::endl;

        cloud_dst->width = 1;
        cloud_dst->height = cloud_dst->points.size();
        cloud_src->width = 1;
        cloud_src->height = cloud_src->points.size();
        Final.width = 1;
        Final.height = Final.points.size();

        // std::string dst_pcd_filename = "/home/hong/Desktop/pointCloud/dst.pcd";
        // pcl::io::savePCDFileASCII(dst_pcd_filename, *cloud_dst);
        // std::string src_pcd_filename = "/home/hong/Desktop/pointCloud/src.pcd";
        // pcl::io::savePCDFileASCII(src_pcd_filename, *cloud_src);
        // std::string aligned_pcd_filename = "/home/hong/Desktop/pointCloud/aligned.pcd";
        // pcl::io::savePCDFileASCII(aligned_pcd_filename, Final);
    }
    else
    {
        std::cout << "ICP RANSAC has not converged." << std::endl;
    }

    cv_rt = preAlignT * cv_rt;

    return cv_rt;
}

cv::Mat estimatePointCloudMotionFPFH(const ConfigLoader config_loader,
                                     cv::Mat &prev,
                                     cv::Mat &curr, bool &trackOK)
{
    const int skip = 1;
    const float searchRadius = 10.0;
    trackOK = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dst(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_dst->points.resize(prev.rows);
    for (size_t i = 0; i < cloud_dst->points.size(); ++i)
    {
        if (i % skip == 0)
        {
            cloud_dst->points[i].x = prev.at<double>(i, 0);
            cloud_dst->points[i].y = prev.at<double>(i, 1);
            cloud_dst->points[i].z = 0.0;
        }
    }

    cloud_src->points.resize(curr.rows);
    for (size_t i = 0; i < cloud_src->points.size(); ++i)
    {
        if (i % skip == 0)
        {
            cloud_src->points[i].x = curr.at<double>(i, 0);
            cloud_src->points[i].y = curr.at<double>(i, 1);
            cloud_src->points[i].z = 0.0;
        }
    }

    // Keypoint detector
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector_src;
    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector_dst;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_src(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_dst(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_dst(new pcl::search::KdTree<pcl::PointXYZ>());
    iss_detector_src.setSearchMethod(tree_src);
    iss_detector_src.setSalientRadius(searchRadius);
    iss_detector_src.setNonMaxRadius(8);
    iss_detector_src.setThreshold21(0.2);
    iss_detector_src.setThreshold32(0.2);
    iss_detector_src.setMinNeighbors(5);
    iss_detector_src.setNumberOfThreads(8);
    iss_detector_src.setInputCloud(cloud_src);
    iss_detector_src.compute((*keypoints_src));

    iss_detector_dst.setSearchMethod(tree_dst);
    iss_detector_dst.setSalientRadius(searchRadius);
    iss_detector_dst.setNonMaxRadius(8);
    iss_detector_dst.setThreshold21(0.2);
    iss_detector_dst.setThreshold32(0.2);
    iss_detector_dst.setMinNeighbors(5);
    iss_detector_dst.setNumberOfThreads(8);
    iss_detector_dst.setInputCloud(cloud_dst);
    iss_detector_dst.compute((*keypoints_dst));

    cloud_dst->width = 1;
    cloud_dst->height = cloud_dst->points.size();
    cloud_src->width = 1;
    cloud_src->height = cloud_src->points.size();
    keypoints_dst->width = 1;
    keypoints_dst->height = keypoints_dst->points.size();
    keypoints_src->width = 1;
    keypoints_src->height = keypoints_src->points.size();

    std::string dst_pcd_filename = "/home/hong/Desktop/pointCloud/dst.pcd";
    pcl::io::savePCDFileASCII(dst_pcd_filename, *cloud_dst);
    std::string src_pcd_filename = "/home/hong/Desktop/pointCloud/src.pcd";
    pcl::io::savePCDFileASCII(src_pcd_filename, *cloud_src);
    std::string dst_kp_filename = "/home/hong/Desktop/pointCloud/dst_kp.pcd";
    pcl::io::savePCDFileASCII(dst_kp_filename, *keypoints_dst);
    std::string src_kp_filename = "/home/hong/Desktop/pointCloud/src_kp.pcd";
    pcl::io::savePCDFileASCII(src_kp_filename, *keypoints_src);

    // // To store normals
    // pcl::PointCloud<pcl::Normal>::Ptr normals_src(new pcl::PointCloud<pcl::Normal>());
    // pcl::PointCloud<pcl::Normal>::Ptr normals_dst(new pcl::PointCloud<pcl::Normal>());

    // // Create the FPFH estimation class, and pass the input dataset+normals to it
    // pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_src;
    // pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_dst;
    // fpfh_src.setInputCloud(cloud_src);
    // fpfh_src.setInputNormals(normals_src);
    // fpfh_dst.setInputCloud(cloud_dst);
    // fpfh_dst.setInputNormals(normals_dst);
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_src(new pcl::search::KdTree<pcl::PointXYZ>);
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_dst(new pcl::search::KdTree<pcl::PointXYZ>);
    // fpfh_src.setSearchMethod(tree_src);
    // fpfh_dst.setSearchMethod(tree_dst);

    // // Output features
    // pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
    // pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_dst(new pcl::PointCloud<pcl::FPFHSignature33>());

    // // Use all neighbors in a sphere of radius 5cm
    // fpfh_src.setRadiusSearch(searchRadius);
    // fpfh_dst.setRadiusSearch(searchRadius);

    // // Compute the features
    // fpfh_src.compute(*fpfhs_src);
    // fpfh_dst.compute(*fpfhs_dst);
}

/*  Finding the maximum clique on a graph with adjacency matrix W.
    Reference: https://www-robotics.jpl.nasa.gov/publications/Andrew_Howard/howard_iros08_visodom.pdf*/
std::vector<cv::DMatch> findMaximumInliers(const float dist_thres, const std::vector<cv::DMatch> unary_matches,
                                           std::vector<cv::Point2f> queryPoints, std::vector<cv::Point2f> trainPoints)
{
    std::vector<cv::DMatch> matches;
    std::set<int> vertex;
    std::multimap<int, int> edges;
    // std::cout << "clique distance threshold is: " << dist_thres << std::endl;
    // std::cout << "unary_matches size is: " << unary_matches.size() << std::endl;

    // // Compute the adjacency matrix W
    // int W_consist[unary_matches.size()][unary_matches.size()] = {0}; // all elements 0

    for (int i = 0; i < unary_matches.size() - 1; i++)
    {
        // Compute the adjacency matrix W
        float u_i_query, v_i_query, u_i_train, v_i_train;
        u_i_query = queryPoints[unary_matches[i].queryIdx].x;
        v_i_query = queryPoints[unary_matches[i].queryIdx].y;
        u_i_train = trainPoints[unary_matches[i].trainIdx].x;
        v_i_train = trainPoints[unary_matches[i].trainIdx].y;

        for (int j = i + 1; j < unary_matches.size() - 1; j++)
        {

            float u_j_query, v_j_query, u_j_train, v_j_train;
            u_j_query = queryPoints[unary_matches[j].queryIdx].x;
            v_j_query = queryPoints[unary_matches[j].queryIdx].y;
            u_j_train = trainPoints[unary_matches[j].trainIdx].x;
            v_j_train = trainPoints[unary_matches[j].trainIdx].y;

            // float dist_pos_thres = 10;

            // float dist_pos_i = abs(
            //     sqrt(pow((u_i_query - u_i_train), 2) + pow((v_i_query - v_i_train), 2)));

            // float dist_pos_j = abs(
            //     sqrt(pow((u_j_query - u_j_train), 2) + pow((v_j_query - v_j_train), 2)));

            // if (dist_pos_i > dist_pos_thres || dist_pos_j > dist_pos_thres)
            //     continue;

            float dist_clique = fabs(
                sqrt(pow((u_i_query - u_j_query), 2) + pow((v_i_query - v_j_query), 2)) -
                sqrt(pow((u_i_train - u_j_train), 2) + pow((v_i_train - v_j_train), 2)));

            if (dist_clique < dist_thres)
            {
                // std::cout << "dist_clique: " << dist_clique << std::endl;
                vertex.insert(i);
                vertex.insert(j);
                edges.insert(std::make_pair(i, j));
            }
        }
    }

    // This should not happen
    if (edges.size() < 3)
    {
        std::cout << "clique edges size less than 3." << std::endl;
        return matches;
    }

    int qsize = 0;
    int *qmax = findMaximumClique(vertex, edges, qsize);
    // std::cout << "maximum clique qsize: " << qsize << std::endl;
    for (int i = 0; i < qsize; i++)
    {
        matches.push_back(unary_matches[qmax[i]]);
    }

    return matches;
}

int *findMaximumClique(std::set<int> v, std::multimap<int, int> e, int &_qsize)
{

    bool **conn;
    int size;

    //  size = v.size() + 1;
    size = *v.rbegin() + 1;
    conn = new bool *[size];
    for (int i = 0; i < size; i++)
    {
        conn[i] = new bool[size];
        memset(conn[i], 0, size * sizeof(bool));
    }
    for (std::multimap<int, int>::iterator it = e.begin(); it != e.end(); it++)
    {
        conn[it->first][it->second] = true;
        conn[it->second][it->first] = true;
    }
    // std::cout << "|E| = " << e.size() << "  |V| = " << v.size() << " p = " << (double)e.size() / (v.size() * (v.size() - 1) / 2) << std::endl;

    clock_t start1 = time(NULL);
    clock_t start2 = clock();
    Maxclique m(conn, size);
    int *qmax;
    int qsize;

    Maxclique md(conn, size, 0.8); //(3rd parameter is optional - default is 0.025 - this heuristics parameter enables you to use dynamic resorting of vertices (time expensive)
    md.mcqdyn(qmax, qsize);        // run max clique with improved coloring and dynamic sorting of vertices
    _qsize = qsize;
    return qmax;
}

// Detect keypoints
void detectKeypoint(const ConfigLoader config_loader, cv::Mat cart_image, std::vector<cv::KeyPoint> &keypoints)
{
    int rows = config_loader.cartImageHeight;
    int cols = config_loader.cartImageWidth;
    double scale = config_loader.cartImageScale;

    // Mask for keypoints detection
    cv::Mat mask = cv::Mat(cart_image.size(), CV_8UC1);
    mask = 255;

    // mask out the region plus 2 and minus 2 meter from the center
    int meters = config_loader.centerMaskRange;
    int offset = round(cols / 2.0 / scale) * meters;
    // std::cout << "pixel offset: " << offset << std::endl;
    mask.colRange(round(cols / 2.0) - offset, round(cols / 2.0) + offset) = 0;

    // detect keypoints
    int minHessian = config_loader.minHessian;
    int nOctaves = 4;
    int nOctaveLayers = 3;
    bool extended = false;
    bool upright = true;
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian, nOctaves, nOctaveLayers, extended, upright);
    detector->detect(cart_image, keypoints, mask);
}

// Compute SURF descriptors
void extractSurfDescriptor(const ConfigLoader config_loader, cv::Mat image, std::vector<cv::KeyPoint> keypoints, cv::Mat &descriptors)
{

    cv::Ptr<cv::xfeatures2d::SURF> extractor = cv::xfeatures2d::SURF::create();
    // Calculate descriptors (feature vectors)
    extractor->compute(image, keypoints, descriptors);
}

// keypoints matching
void ImageKeypointMatching(const ConfigLoader mConfigLoader, cv::Mat cart_image,
                           std::vector<cv::KeyPoint> keypoints1, std::vector<cv::KeyPoint> keypoints2,
                           cv::Mat descriptors1, cv::Mat descriptors2,
                           std::vector<cv::Point2f> &curr, std::vector<cv::Point2f> &prev,
                           std::vector<cv::DMatch> &matches)
{
    std::vector<cv::DMatch> _matches;
    std::cout << "size: " << keypoints1.size() << " of keypoints1." << std::endl;
    std::cout << "size: " << keypoints2.size() << " of keypoints2." << std::endl;
    std::cout << "rows: " << descriptors1.rows << " and cols: " << descriptors1.cols << " of descriptors 1." << std::endl;
    std::cout << "rows: " << descriptors2.rows << " and cols: " << descriptors2.cols << " of descriptors 2." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    // std::cout << "mConfigLoader.treeRadius: " << mConfigLoader.treeRadius << std::endl;

    if (mConfigLoader.matcherOption == 0)
    {

        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = mConfigLoader.LowesRatio;

        // ------------------------------------------------Neighbourhood search---------------------------------------------------

        auto start1 = std::chrono::high_resolution_clock::now();

        // KD-tree of keypoints in previous image frame
        pointVec points;
        point_t pt;
        for (int i = 0; i < keypoints2.size(); i++)
        {
            cv::Point2d kp_prev = keypoints2[i].pt;
            pt = {kp_prev.x, kp_prev.y};
            points.push_back(pt);
        }
        auto startTree = std::chrono::high_resolution_clock::now();
        KDTree Tree_prev(points);
        auto stopTree = std::chrono::high_resolution_clock::now();
        auto durationTree = std::chrono::duration_cast<std::chrono::milliseconds>(stopTree - startTree);
        // std::cout << "Tree construction computation time is: " << durationTree.count() << " milliseconds." << std::endl;

        // Search matches in neighbourhood
        std::vector<cv::Point2f> curr_refined, prev_refined;
        std::vector<cv::DMatch> matches_neighbour;
        std::set<size_t> used_train_set;
        for (int i = 0; i < keypoints1.size(); ++i)
        {
            cv::KeyPoint pt_curr = keypoints1[i];

            cv::Mat pt_homo = cv::Mat(3, 1, CV_32FC1);
            pt_homo.at<float>(0, 0) = pt_curr.pt.x;
            pt_homo.at<float>(1, 0) = pt_curr.pt.y;
            pt_homo.at<float>(2, 0) = 1.0;
            // pt_homo = H*pt_homo;

            point_t transformed_pt = {pt_homo.at<float>(0, 0), pt_homo.at<float>(1, 0)};
            indexArr neighborhood_indices = Tree_prev.neighborhood_indices(transformed_pt, mConfigLoader.treeRadius);
            // std::cout << "neighborhoood_indices size: " << neighborhood_indices.size() << std::endl;
            if (neighborhood_indices.size() < 1)
                continue;

            // Find the best match
            double TopOneDist = 10000.0;
            double TopTwoDist = 10001.0;
            size_t minIndex = -1;
            cv::Mat descriptor_curr = descriptors1.row(i);
            for (int j = 0; j < neighborhood_indices.size(); j++)
            {
                double dist;
                size_t index = neighborhood_indices[j];
                cv::Mat descriptor_prev = descriptors2.row(index);
                dist = cv::norm(descriptor_curr, descriptor_prev, cv::NORM_L2); // l2 for surf,sift
                if (dist < TopOneDist)
                {
                    TopTwoDist = TopOneDist;
                    TopOneDist = dist;
                    minIndex = index;
                }
                else if (dist < TopTwoDist)
                {
                    TopTwoDist = dist;
                }
                else
                {
                    // Greater than both distances.
                }
            }

            // check if the matche index already been taken by other query point
            if (used_train_set.count(minIndex))
                continue;

            if (TopOneDist < ratio_thresh * TopTwoDist)
            {
                used_train_set.insert(minIndex);
                cv::DMatch good_match(i, minIndex, TopOneDist);
                // std::cout << "current point x: " << keypoints1[i].pt.x << " y: " << keypoints1[i].pt.y << std::endl;
                // std::cout << "previous point x: " << keypoints2[minIndex].pt.x << " y: " << keypoints2[minIndex].pt.y << std::endl;
                matches_neighbour.push_back(good_match);
            }
        }
        auto stop1 = std::chrono::high_resolution_clock::now();
        auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start1);
        // std::cout << "Neighbourhood search computation time is: " << duration1.count() << " milliseconds." << std::endl;
        // std::cout << "Number of Neighbourhood matches is: " << matches_neighbour.size() << std::endl;

        _matches = matches_neighbour;
    }
    else if (mConfigLoader.matcherOption == 1)
    {
        printf("FLANN MATHER!\n");

        //-- Matching descriptor vectors with a FLANN based matcher
        // Since SURF is a floating-point descriptor NORM_L2 is used
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descriptors1, descriptors2, knn_matches, 2);
        // std::cout << "Number of knn_matches is: " << knn_matches.size() << std::endl;

        //-- Filter matches using the Lowe's ratio test
        const float ratio_thresh = mConfigLoader.LowesRatio;
        std::vector<cv::DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++)
        {
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
            {
                good_matches.push_back(knn_matches[i][0]);
            }
        }
        // std::cout << "Number of FLANN matches is: " << good_matches.size() << std::endl;

        // Get those "good" matches~
        _matches = good_matches;
    }
    else if (mConfigLoader.matcherOption == 2)
    {
        printf("BRUTE FORCE MATCHER!\n");
        cv::BFMatcher matcher(cv::NORM_L2, true);
        matcher.match(descriptors1, descriptors2, _matches);
    }
    else
    {
        std::cerr << "Error in the matching option." << std::endl;
    }

    // std::cout << "Number of matches before outlier rejection: " << _matches.size() << std::endl;

    //----------------------------------------------------------------------------------------------------------------------------------

    //-- Localize the object
    for (int i = 0; i < _matches.size(); i++)
    {
        //-- Get the keypoints from the good matches
        curr.push_back(keypoints1[_matches[i].queryIdx].pt);
        prev.push_back(keypoints2[_matches[i].trainIdx].pt);
    }

    // -------------------------------------------Homography to reject outliers-------------------------------------------
    auto start2 = std::chrono::high_resolution_clock::now();
    std::vector<cv::Point2f> curr_homo, prev_homo;
    std::vector<cv::DMatch> matches_homo;
    cv::Mat mask;
    cv::Mat H_ = cv::findHomography(curr, prev, CV_RANSAC, mConfigLoader.treeRadius, mask);
    for (int i = 0; i < curr.size(); i++)
    {
        if (mask.at<uchar>(i, 0))
        {
            matches_homo.push_back(_matches[i]);
            curr_homo.push_back(keypoints1[_matches[i].queryIdx].pt);
            prev_homo.push_back(keypoints2[_matches[i].trainIdx].pt);
        }
    }

    curr = curr_homo;
    prev = prev_homo;
    _matches = matches_homo;
    std::cout << "Number of matches before Maximum Cliques: " << _matches.size() << std::endl;
    auto stop2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(stop2 - start2);
    std::cout << "Homography rejection computation time is: " << duration2.count() << " milliseconds." << std::endl;

    // ---------------------------------------------------Clique matches criteria-----------------------------------------------------
    auto start3 = std::chrono::high_resolution_clock::now();
    std::vector<cv::DMatch> matches_clique;
    if (_matches.size() > mConfigLoader.minMatchesBetweenFrames)
    {
        std::vector<cv::Point2f> points1, points2;
        for (int i = 0; i < keypoints1.size(); i++)
        {
            points1.push_back(keypoints1[i].pt);
        }
        for (int i = 0; i < keypoints2.size(); i++)
        {
            points2.push_back(keypoints2[i].pt);
        }
        matches_clique = findMaximumInliers(mConfigLoader.cliqueDistThres, _matches, points1, points2);

        std::vector<cv::Point2f> curr_clique, prev_clique;
        // std::cout << "Number of matches_clique is: " << matches_clique.size() << std::endl;
        for (size_t i = 0; i < matches_clique.size(); ++i)
        {
            //-- Get the keypoints from the clique matches
            curr_clique.push_back(keypoints1[matches_clique[i].queryIdx].pt);
            prev_clique.push_back(keypoints2[matches_clique[i].trainIdx].pt);
        }
        curr = curr_clique;
        prev = prev_clique;
    }

    if (matches_clique.size() > mConfigLoader.minMatchesBetweenFrames)
    {
        _matches = matches_clique;
        std::cout << "Number of clique matches is: " << matches_clique.size() << std::endl;
    }

    auto stop3 = std::chrono::high_resolution_clock::now();
    auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds>(stop3 - start3);
    std::cout << "Maximum clique computation time is: " << duration3.count() << " milliseconds." << std::endl;

    // ---------------------------------------------------Clique matches criteria-----------------------------------------------------

    std::cout << "Number of matches at the end: " << _matches.size() << std::endl;
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Matching computation time is: " << duration.count() << " milliseconds." << std::endl;

    matches = _matches;
}

void DynamicTargetRemoval(const ConfigLoader config_loader, std::vector<cv::DMatch> &matches,
                          std::vector<cv::Point2f> prevPoints, std::vector<cv::Point2f> currpoints)
{

    // auto start = std::chrono::high_resolution_clock::now();

    // std::cout << "Number of points: " << prevPoints.size() << std::endl;

    std::vector<cv::DMatch> matches_clique = findMaximumInliers(config_loader.cliqueDistThres, matches, currpoints, prevPoints);
    // std::cout << "Initial number of matches: " << matches.size() << " and after clique: " << matches_clique.size() << std::endl;
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // std::cout << "Dynamics removal computation time is: " << duration.count() << " milliseconds." << std::endl;

    matches = matches_clique;
}

void FeatureExtractionANMS(cv::Mat cart_image, ConfigLoader config_loader, std::vector<cv::KeyPoint> &keypoints, cv::Mat &descriptors)
{

    // auto start = std::chrono::high_resolution_clock::now();

    int rows = config_loader.cartImageHeight;
    int cols = config_loader.cartImageWidth;
    double scale = config_loader.cartImageScale;

    // Mask for keypoints detection
    cv::Mat mask = cv::Mat(cart_image.size(), CV_8UC1);
    mask = 255;

    // mask out the region plus 2 and minus 2 meter from the center
    int meters = config_loader.centerMaskRange;
    int offset = round(cols / 2.0 / scale) * meters;
    // std::cout << "pixel offset: " << offset << std::endl;
    mask.colRange(round(cols / 2.0) - offset, round(cols / 2.0) + offset) = 0;

    // detect keypoints
    int minHessian = config_loader.minHessian;
    int nOctaves = 4;
    int nOctaveLayers = 3;
    bool extended = false;
    bool upright = true;
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian, nOctaves, nOctaveLayers, extended, upright);
    detector->detect(cart_image, keypoints, mask);

    /** \brief Start of Aplly ANMS to non-maximal suppresion
     * */
    std::vector<float> responseVector;
    for (unsigned int i = 0; i < keypoints.size(); i++)
        responseVector.push_back(keypoints[i].response);
    std::vector<int> Indx(responseVector.size());
    std::iota(std::begin(Indx), std::end(Indx), 0);
    cv::sortIdx(responseVector, Indx, CV_SORT_DESCENDING);
    std::vector<cv::KeyPoint> keyPointsSorted;
    for (unsigned int i = 0; i < keypoints.size(); i++)
        keyPointsSorted.push_back(keypoints[Indx[i]]);
    int numRetPoints = config_loader.minimumPointsANMS; //choose exact number of return points
    if (numRetPoints > keyPointsSorted.size())
        numRetPoints = keyPointsSorted.size();
    // float tolerance = 0.1;  // Default: tolerance of the number of return points
    float tolerance = 0.1; // tolerance of the number of return points

    std::vector<cv::KeyPoint> sscKP = Ssc(keyPointsSorted, numRetPoints, tolerance, cart_image.cols, cart_image.rows);
    keypoints = sscKP;
    /** \brief End of Aplly ANMS to non-maximal suppresion
     * */


    // Describe SURF descriptor
    // detector->compute(cart_image, keypoints, descriptors);

    // Resize descriptors to fool the system
    descriptors = cv::Mat(keypoints.size(), 1, CV_8UC1);

    // std::cout << "Generated " << keypoints.size() << " points using ANMS." << std::endl;
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // std::cout << "Feature extraction time using ANMS: " << duration.count() << " ms." << std::endl;
}