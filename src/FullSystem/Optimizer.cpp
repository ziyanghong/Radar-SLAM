#include "Optimizer.h"



Optimizer::Optimizer()
{
}

Optimizer::~Optimizer()
{
}



void Optimizer::GlobalBundleAdjustment(Map *pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                       const bool bRobust, const ConfigLoader *pConfigLoader)
{
    std::vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
    std::vector<MapPoint *> vpMP = pMap->GetAllMapPoints();

    // Correct all the map points before global BA.
    // We assume here we have performed essential graph optimization prior to that.
    size_t deletedMP = 0;
    for (size_t i = 0; i < vpMP.size(); i++)
    {
        MapPoint *pMP = vpMP[i];
        if (pMP->isBad())
            continue;
        if (pMP->nObs < 3)
        {
            pMap->DeleteMapPoint(pMP);
            deletedMP++;
            continue;
        }

        const std::map<KeyFrame *, size_t> observations = pMP->GetObservations();
        std::map<KeyFrame *, size_t>::const_iterator mit = observations.begin();
        KeyFrame *pKFi = mit->first;
        size_t nKeyPoint = mit->second;
        cv::Point2d image_point = pKFi->mvKeypoints[nKeyPoint].pt;
        cv::Point2d obs_xy = imageFrame2LocalFrame(pConfigLoader->cartImageHeight, pConfigLoader->cartImageWidth,
                                                   pConfigLoader->cartImageScale, image_point);

        cv::Mat LocalPos = cv::Mat(3, 1, CV_32FC1);
        LocalPos.at<float>(0, 0) = obs_xy.x;
        LocalPos.at<float>(1, 0) = obs_xy.y;
        LocalPos.at<float>(2, 0) = 1.0;
        cv::Mat WorldPos = pKFi->GetPose() * LocalPos;
        pMP->SetWorldPos(WorldPos);
    }
    // std::cout << "Number of deleted map points: " << deletedMP << std::endl;

    // Take the updated map points
    vpMP = pMap->GetAllMapPoints();
    BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust, pConfigLoader);
}

void Optimizer::BundleAdjustment(const std::vector<KeyFrame *> &vpKFs, const std::vector<MapPoint *> &vpMP,
                                 int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust,
                                 const ConfigLoader *pConfigLoader)
{
    const ConfigLoader config_loader = *pConfigLoader;

    // Define Solver
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
    typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // Initialize Solver
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;

    // Define a robust kernel
    g2o::RobustKernel *robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");

    // Set KeyFrame vertices
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKFi = vpKFs[i];
        cv::Mat Tcw = pKFi->GetPose();
        double tangent = Tcw.at<float>(1, 0) / Tcw.at<float>(0, 0);
        double angle_yaw = atan2(Tcw.at<float>(1, 0), Tcw.at<float>(0, 0));
        g2o::SE2 pose(Tcw.at<float>(0, 2), Tcw.at<float>(1, 2), angle_yaw);
        g2o::VertexSE2 *vSE2 = new g2o::VertexSE2();
        vSE2->setEstimate(pose);
        vSE2->setId(pKFi->mnId);
        vSE2->setFixed(pKFi->mnId == 0);
        optimizer.addVertex(vSE2);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // Set Map Point vertices
    std::vector<g2o::EdgeSE2PointXY *> vEdges;
    for (size_t i = 0; i < vpMP.size(); i++)
    {
        MapPoint *pMP = vpMP[i];
        g2o::VertexPointXY *vertexMapPoint = new g2o::VertexPointXY;
        int id = pMP->mnId + maxKFid + 1;
        vertexMapPoint->setId(id);
        cv::Mat WorldPos = pMP->GetWorldPos();
        Eigen::Vector2d EigenWorldPos(WorldPos.at<float>(0, 0), WorldPos.at<float>(1, 0));
        vertexMapPoint->setEstimate(EigenWorldPos);
        optimizer.addVertex(vertexMapPoint);

        const std::map<KeyFrame *, size_t> observations = pMP->GetObservations();

        //Set edges
        for (std::map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;
            size_t nKeyPoint = mit->second;

            g2o::EdgeSE2PointXY *e = new g2o::EdgeSE2PointXY;
            e->vertices()[0] = optimizer.vertex(pKFi->mnId);
            e->vertices()[1] = optimizer.vertex(id);

            cv::Point2d image_point = pKFi->mvKeypoints[nKeyPoint].pt;
            cv::Point2d obs_xy = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth,
                                                       config_loader.cartImageScale, image_point);
            Eigen::Vector2d measurement(obs_xy.x, obs_xy.y);
            e->setMeasurement(measurement);

            Eigen::Matrix2d information = Eigen::Matrix<double, 2, 2>::Identity();
            information(0, 0) = information(1, 1) = config_loader.landmarkInformation;
            e->setInformation(information);
            if (bRobust)
            {
                e->setRobustKernel(robustKernel);
            }

            optimizer.addEdge(e);
            vEdges.push_back(e);
        }
    }

    std::cout << "Optimizer::GlobalBundleAdjustment: Number of Keyframes to be optimized: " << vpKFs.size() << std::endl;
    std::cout << "Optimizer::GlobalBundleAdjustment: Number of MapPoints to be optimized: " << vpMP.size() << std::endl;

    std::string g2o_result_before = config_loader.g2o_result_path + std::to_string(nLoopKF) + "_globalBA_pose_graph_before.g2o";
    std::string g2o_result_after = config_loader.g2o_result_path + std::to_string(nLoopKF) + "_globalBA_pose_graph_after.g2o";
    optimizer.save(g2o_result_before.c_str());
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);
    optimizer.save(g2o_result_after.c_str());

    //Keyframes
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;

        g2o::VertexSE2 *vSE2_recov = static_cast<g2o::VertexSE2 *>(optimizer.vertex(pKF->mnId));
        g2o::SE2 SE2_recov = vSE2_recov->estimate();
        if (nLoopKF == 0)
        {
            cv::Mat optimized_Tcw = toCvMat(SE2_recov);
            pKF->SetPose(optimized_Tcw);
        }
        else
        {
        }
    }

    //Points
    for (size_t i = 0; i < vpMP.size(); i++)
    {
        MapPoint *pMP = vpMP[i];
        if (pMP->isBad())
            continue;
        g2o::VertexPointXY *vPoint = static_cast<g2o::VertexPointXY *>(optimizer.vertex(pMP->mnId + maxKFid + 1));

        if (nLoopKF == 0)
        {
            Eigen::Vector2d EigenWorldPos = vPoint->estimate();
            pMP->SetWorldPos(toCvMat(EigenWorldPos));
        }
        else
        {
        }
    }
    std::cout << "Map updated!" << std::endl;
}

/**
 * 
 * Optimize essential graph when loop detected
 */
void Optimizer::OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                       const cv::Mat &LoopRT, const cv::Mat &Tcm,
                                       ConfigLoader config_loader)
{
    auto start = std::chrono::high_resolution_clock::now();

    // Define Solver
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
    // typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;


    // Initialize Solver
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);

    g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(
        g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    // optimizer.setVerbose(true);
    double numberOfEdges = 1;
    const double chi2Factor = 3.0;
    const int numberOfIteration = 5;

    // Get all the keyframes
    const std::vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
    int totalNumberMatches = 0;
    for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
    {
        KeyFrame *pKF = vpKFs[i];
        totalNumberMatches = totalNumberMatches + pKF->GetNumberMatchesWithLastKeyFrame();
    }
    double averageNumberMatches = totalNumberMatches / vpKFs.size();

    // Set KeyFrame vertices
    for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
    {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;

        cv::Mat Tcw = pKF->GetPose();
        double angle_yaw = atan2(Tcw.at<float>(1, 0), Tcw.at<float>(0, 0));
        g2o::SE2 pose(Tcw.at<float>(0, 2), Tcw.at<float>(1, 2), angle_yaw);
        g2o::VertexSE2 *vSE2 = new g2o::VertexSE2();
        vSE2->setEstimate(pose);
        vSE2->setId(pKF->mnId);
        vSE2->setFixed(pKF->mnId == 0); // fix the first robot pose to account for gauge freedom

        if (pKF == pLoopKF)
        {
            vSE2->setFixed(true);
            // std::cout << "Fixed the vertex of loop keyframe." << std::endl;
        }

        if (pKF == pCurKF)
        {
            angle_yaw = atan2(Tcm.at<float>(1, 0), Tcm.at<float>(0, 0));
            g2o::SE2 poseCorrected(Tcm.at<float>(0, 2), Tcm.at<float>(1, 2), angle_yaw);
            vSE2->setEstimate(poseCorrected);
        }

        optimizer.addVertex(vSE2);
    }

    // Set loop edges
    g2o::EdgeSE2 *edgeNew = new g2o::EdgeSE2();
    edgeNew->vertices()[0] = optimizer.vertex(pLoopKF->mnId);
    edgeNew->vertices()[1] = optimizer.vertex(pCurKF->mnId);
    // std::cout << "Tcm: " << std::endl << Tcm<< std::endl;
    double angle_yaw_loop = atan2(LoopRT.at<float>(1, 0), LoopRT.at<float>(0, 0));

    g2o::SE2 Scm(LoopRT.at<float>(0, 2), LoopRT.at<float>(1, 2), angle_yaw_loop);
    edgeNew->setMeasurement(Scm);

    // Information matrix for loop closure
    // double factor = config_loader.loopFactor;
    double factor = 1.0;

    Eigen::Matrix<double, 3, 3> informationLoop = Eigen::Matrix<double, 3, 3>::Identity();
    // factor = pCurKF->GetNumberMatchesWithLoop() / averageNumberMatches;
    informationLoop(0, 0) = informationLoop(1, 1) = config_loader.poseInformationXY * factor;
    informationLoop(2, 2) = config_loader.poseInformationYaw * factor;
    edgeNew->setInformation(informationLoop);
    optimizer.addEdge(edgeNew);

    // Set normal edges including the parent edges and previous loop edges
    for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
    {
        KeyFrame *pKF = vpKFs[i];

        const int nIDi = pKF->mnId;
        cv::Mat Tiw = pKF->GetPose();
        double angle_yaw_i = atan2(Tiw.at<float>(1, 0), Tiw.at<float>(0, 0));
        g2o::SE2 Siw(Tiw.at<float>(0, 2), Tiw.at<float>(1, 2), angle_yaw_i);

        // Spinning tree edge
        KeyFrame *pParentKF = pKF->GetParent();

        if (pParentKF)
        {
            int nIDj = pParentKF->mnId;
            cv::Mat Tjw = pParentKF->GetPose();
            double angle_yaw_j = atan2(Tjw.at<float>(1, 0), Tjw.at<float>(0, 0));
            g2o::SE2 Sjw(Tjw.at<float>(0, 2), Tjw.at<float>(1, 2), angle_yaw_j);
            const g2o::SE2 Swj = Sjw.inverse();

            g2o::EdgeSE2 *edge = new g2o::EdgeSE2();
            edge->vertices()[0] = optimizer.vertex(nIDj);
            edge->vertices()[1] = optimizer.vertex(nIDi);
            const g2o::SE2 Sji = Swj * Siw; // Swj * Sji = Swi, Sji = Swj.inv * Swi
            edge->setMeasurement(Sji);
            Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity(); // Information matrix
            information(0, 0) = information(1, 1) = config_loader.poseInformationXY;
            information(2, 2) = config_loader.poseInformationYaw;
            edge->setInformation(information);
            // edge->setRobustKernel(robustKernel);
            optimizer.addEdge(edge);
            numberOfEdges++;
        }

        // Previous Loop edges
        const std::set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
        for (std::set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++)
        {
            KeyFrame *pLKF = *sit;

            cv::Mat Tlw = pLKF->GetPose();
            double angle_yaw_j = atan2(Tlw.at<float>(1, 0), Tlw.at<float>(0, 0));
            g2o::SE2 Slw(Tlw.at<float>(0, 2), Tlw.at<float>(1, 2), angle_yaw_j);
            const g2o::SE2 Swl = Slw.inverse();
            g2o::SE2 Sli = Swl * Siw;

            // Set nodes
            g2o::EdgeSE2 *el = new g2o::EdgeSE2();
            el->vertices()[0] = optimizer.vertex(pLKF->mnId);
            el->vertices()[1] = optimizer.vertex(nIDi);
            el->setMeasurement(Sli);

            // Set information
            double factorLoop = 1;
            Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity(); // Information matrix
            factorLoop = pLKF->GetNumberMatchesWithLoop() / averageNumberMatches;
            information(0, 0) = information(1, 1) = config_loader.poseInformationXY * factorLoop;
            information(2, 2) = config_loader.poseInformationYaw * factorLoop;
            el->setInformation(information);
            // el->setRobustKernel(robustKernel);
            optimizer.addEdge(el);
            numberOfEdges++;
        }
    }

    /*************************************************************************************************
     *                                            optimization
     * **********************************************************************************************/

    // Optimize
    // std::string g2o_result_path = "/home/hong/Documents/g2o/radar_localization_graph/";
    // std::string g2o_result_before = g2o_result_path + std::to_string(pCurKF->mnFrameId) + "_loop_pose_graph_before.g2o";
    // std::string g2o_result_after = g2o_result_path + std::to_string(pCurKF->mnFrameId) + "_loop_pose_graph_after.g2o";
    // optimizer.save(g2o_result_before.c_str());
    optimizer.initializeOptimization();
    // auto stop1 = std::chrono::high_resolution_clock::now();
    // auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start);
    // std::cout << "Pose graph initializeOptimization time: " << duration1.count() << " ms." << std::endl;
    // auto start1 = std::chrono::high_resolution_clock::now();
    optimizer.optimize(numberOfIteration);
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start1);
    // std::cout << "Pose graph optimization time: " << duration.count() << " ms." << std::endl;

    // optimizer.save(g2o_result_after.c_str());

    // Check Chi2 error, if too large, don't correct pose
    double activeChi2 = optimizer.activeChi2();
    if (activeChi2 > numberOfEdges * chi2Factor)
        return;

    // Lock to update pose
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

    // SE2 pose recovering.
    for (size_t i = 0; i < vpKFs.size(); i++)
    {
        KeyFrame *pKFi = vpKFs[i];
        g2o::VertexSE2 *vSE2_recov = static_cast<g2o::VertexSE2 *>(optimizer.vertex(pKFi->mnId));
        g2o::SE2 SE2_recov = vSE2_recov->estimate();
        // std::cout << "KF mnId " << pKFi->mnId << " SE2_recov after optimization: " << std::endl << SE2_recov.toVector() << std::endl;
        cv::Mat optimized_Tcw = toCvMat(SE2_recov);

        // Update with the optimized pose
        pKFi->SetPose(optimized_Tcw);
    }

    // std::cout << "Pose of current KF after essentialGraph optimization: " << std::endl
    //           << pCurKF->GetPose() << std::endl;

    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // std::cout << "Pose graph optimization time: " << duration.count() << " ms." <<std::endl;

    // std::cout << "Finished OptimizeEssentialGraph." << std::endl;
    // std::cout << "Press Enter to continue...\n" << std::endl;
    // std::getchar();
}

/**
 * Optimize local map when new keyframe is added to the map
 */
void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, ConfigLoader config_loader)
{
    // printf("Enter LocalBundleAdjustment\n");

    // Local KeyFrames: First Search from Current Keyframe
    std::list<KeyFrame *> lLocalKeyFrames;
    lLocalKeyFrames.push_back(pKF);

    const std::vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
    {
        KeyFrame *pKFi = vNeighKFs[i];
        lLocalKeyFrames.push_back(pKFi);
    }

    // Local MapPoints seen in Local KeyFrames
    std::list<MapPoint *> lLocalMapPoints;
    for (std::list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(); lit != lLocalKeyFrames.end(); lit++)
    {
        std::vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
        for (std::vector<MapPoint *>::iterator vit = vpMPs.begin(); vit != vpMPs.end(); vit++)
        {
            MapPoint *pMP = *vit;
            if (pMP)
                if (!pMP->isBad())
                    if (pMP->mnBALocalForKF != pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                        pMP->mbUpdated = true;
                        pMap->mvpUpdatedMapPoints.push_back(pMP); // For viewer update
                    }
        }
    }

    // std::cout << "Optimizer::LocalBundleAdjustment: Number of Keyframes to be optimized: " << lLocalKeyFrames.size() << std::endl;
    // std::cout << "Optimizer::LocalBundleAdjustment: Number of MapPoints to be optimized: " << lLocalMapPoints.size() << std::endl;

    // This should not happen except for the first keyframe
    if (lLocalKeyFrames.size() == 0)
        return;

    // Define Solver
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
    typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // Initialize Solver
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;

    // Define a robust kernel
    g2o::RobustKernel *robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");

    // Set Local KeyFrame vertices
    for (std::list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
    {
        KeyFrame *pKFi = *lit;
        cv::Mat Tcw = pKFi->GetPose();
        double tangent = Tcw.at<float>(1, 0) / Tcw.at<float>(0, 0);
        double angle_yaw = atan2(Tcw.at<float>(1, 0), Tcw.at<float>(0, 0));
        g2o::SE2 pose(Tcw.at<float>(0, 2), Tcw.at<float>(1, 2), angle_yaw);
        g2o::VertexSE2 *vSE2 = new g2o::VertexSE2();
        vSE2->setEstimate(pose);
        vSE2->setId(pKFi->mnId);
        vSE2->setFixed(pKFi->mnId == 0);
        optimizer.addVertex(vSE2);
        if (pKFi->mnId > maxKFid)
            maxKFid = pKFi->mnId;
    }

    // Set Local Map Point vertices
    std::vector<g2o::EdgeSE2PointXY *> vEdges;
    for (std::list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
    {
        MapPoint *pMP = *lit;
        g2o::VertexPointXY *vertexMapPoint = new g2o::VertexPointXY;
        int id = pMP->mnId + maxKFid + 1;
        vertexMapPoint->setId(id);
        cv::Mat WorldPos = pMP->GetWorldPos();
        Eigen::Vector2d EigenWorldPos(WorldPos.at<float>(0, 0), WorldPos.at<float>(1, 0));
        vertexMapPoint->setEstimate(EigenWorldPos);
        optimizer.addVertex(vertexMapPoint);

        const std::map<KeyFrame *, size_t> observations = pMP->GetObservations();

        //Set edges
        for (std::map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {
            KeyFrame *pKFi = mit->first;
            size_t nKeyPoint = mit->second;

            g2o::EdgeSE2PointXY *e = new g2o::EdgeSE2PointXY;
            e->vertices()[0] = optimizer.vertex(pKFi->mnId);
            e->vertices()[1] = optimizer.vertex(id);

            cv::Point2d image_point = pKFi->mvKeypoints[nKeyPoint].pt;
            cv::Point2d obs_xy = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth,
                                                       config_loader.cartImageScale, image_point);
            Eigen::Vector2d measurement(obs_xy.x, obs_xy.y);
            e->setMeasurement(measurement);

            Eigen::Matrix2d information = Eigen::Matrix<double, 2, 2>::Identity();
            information(0, 0) = information(1, 1) = config_loader.landmarkInformation;
            e->setInformation(information);

            e->setRobustKernel(robustKernel);

            optimizer.addEdge(e);
            vEdges.push_back(e);
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    // Get Map Mutex
    std::unique_lock<std::mutex> lock(pMap->mMutexMapUpdate);

    /* Recover optimized data */

    //Keyframes
    for (std::list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(); lit != lLocalKeyFrames.end(); lit++)
    {
        KeyFrame *pKF = *lit;
        g2o::VertexSE2 *vSE2_recov = static_cast<g2o::VertexSE2 *>(optimizer.vertex(pKF->mnId));
        g2o::SE2 SE2_recov = vSE2_recov->estimate();
        // std::cout << "SE2_recov: " << std::endl << SE2_recov.toVector() << std::endl;
        cv::Mat optimized_Tcw = toCvMat(SE2_recov);
        pKF->SetPose(optimized_Tcw);
    }

    //Points
    for (std::list<MapPoint *>::iterator lit = lLocalMapPoints.begin(); lit != lLocalMapPoints.end(); lit++)
    {
        MapPoint *pMP = *lit;
        g2o::VertexPointXY *vPoint = static_cast<g2o::VertexPointXY *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
        Eigen::Vector2d EigenWorldPos = vPoint->estimate();
        pMP->SetWorldPos(toCvMat(EigenWorldPos));
    }

    // printf("Exit from LocalBundleAdjustment\n");
}

/**
 * g2o::EdgeSE2XYOnlyPose
 * Optimize on the current pose respect to Map points in the tracking thread
 * Map points are freezed not to optimize in this step
 * and they will be updated by the local mapper.
 */
int Optimizer::PoseOptimization(Frame *pFrame, ConfigLoader config_loader)
{

    // printf("Enter PoseOptimization\n");

    // Define Solver
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
    typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // Initialize Solver
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // Define a robust kernel
    g2o::RobustKernel *robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");

    int nInitialCorrespondences = 0;
    int nBad = 0;

    // Set Frame vertex
    cv::Mat Tcw = pFrame->TcwSVD;
    double tangent = Tcw.at<float>(1, 0) / Tcw.at<float>(0, 0);
    double angle_yaw = atan2(Tcw.at<float>(1, 0), Tcw.at<float>(0, 0));
    g2o::SE2 pose(Tcw.at<float>(0, 2), Tcw.at<float>(1, 2), angle_yaw);
    g2o::VertexSE2 *vSE2 = new g2o::VertexSE2();
    vSE2->setEstimate(pose);
    vSE2->setId(0);
    vSE2->setFixed(false);
    optimizer.addVertex(vSE2);

    // Set MapPoint vertices
    const int N = pFrame->N;

    std::vector<g2o::EdgeSE2XYOnlyPose *> vpEdges;
    std::vector<size_t> vnIndexEdge;
    vpEdges.reserve(N);
    vnIndexEdge.reserve(N);

    {
        std::unique_lock<std::mutex> lock(MapPoint::mGlobalMutex);

        for (int i = 0; i < N; i++)
        {
            MapPoint *pMP = pFrame->mvpMapPoints[i];
            if (pMP)
            {
                nInitialCorrespondences++;
                pFrame->mvbOutlier[i] = false;

                g2o::VertexPointXY *v = new g2o::VertexPointXY;
                cv::Mat WorldPos = pMP->GetWorldPos();
                Eigen::Vector2d mapPoint_Tcw(WorldPos.at<float>(0, 0), WorldPos.at<float>(0, 1));
                v->setEstimate(mapPoint_Tcw);

                // Edge between map point and frame
                g2o::EdgeSE2XYOnlyPose *e = new g2o::EdgeSE2XYOnlyPose;
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                cv::Point2d obs_xy = pFrame->mvKeypoints[i].pt;
                // Convert to image point to local frame coordinate
                obs_xy = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth, config_loader.cartImageScale, obs_xy);
                Eigen::Vector2d obs(obs_xy.x, obs_xy.y);
                e->setMeasurement(obs);
                Eigen::Matrix2d information = Eigen::Matrix<double, 2, 2>::Identity();
                information(0, 0) = information(1, 1) = config_loader.landmarkInformation;
                e->setInformation(information);
                e->setRobustKernel(robustKernel); // Set the robust kernel
                e->l2 = v;
                optimizer.addEdge(e);

                vpEdges.push_back(e);
                vnIndexEdge.push_back(i);
            }
        }
        // std::cout << "optimizing pose graph, vertices: " << optimizer.vertices().size() << std::endl;

        // We perform 1 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        for (int it = 0; it < 1; it++)
        {
            // std::string g2o_result_before = config_loader.g2o_result_path + std::to_string(pFrame->mnId) + "_result_before.g2o";
            // std::string g2o_result_after = config_loader.g2o_result_path + std::to_string(pFrame->mnId) + "_result_after.g2o";
            // optimizer.save(g2o_result_before.c_str());
            optimizer.initializeOptimization();
            optimizer.optimize(config_loader.optimizeStep);
            // optimizer.save(g2o_result_after.c_str());

            nBad = 0;

            for (size_t i = 0, iend = vpEdges.size(); i < iend; i++)
            {
                g2o::EdgeSE2XYOnlyPose *e = vpEdges[i];

                const size_t idx = vnIndexEdge[i];

                if (pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > config_loader.chiTwo)
                {
                    e->setLevel(1);
                    pFrame->mvbOutlier[idx] = true;
                    nBad++;
                    // std::cout << "Number of bad edge: " << nBad << std::endl;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                }
            }

            // std::cout << "Number of optimization of current pose:  " << it+1 << std::endl;

            if (optimizer.edges().size() < 10)
                break;
        }
    }

    // for (size_t i = 0, iend = vpEdges.size(); i < iend; i++)
    // {
    //     g2o::EdgeSE2XYOnlyPose *e = vpEdges[i];
    //     const float chi2 = e->chi2();
    //     std::cout << "Edge: " << i << " has chi2 " << chi2 << std::endl;
    // }

    g2o::VertexSE2 *vSE2_recov = static_cast<g2o::VertexSE2 *>(optimizer.vertex(0));
    g2o::SE2 SE2_recov = vSE2_recov->estimate();
    // std::cout << "SE2_recov: " << std::endl << SE2_recov.toVector() << std::endl;
    cv::Mat optimized_Tcw = toCvMat(SE2_recov);

    int nGoodCorrespondences = nInitialCorrespondences - nBad;
    // std::cout << "Number of good edge in this PoseOptimization(): " << nGoodCorrespondences << std::endl;

    // if (nGoodCorrespondences >= 5)
    pFrame->SetPose(optimized_Tcw);

    return nGoodCorrespondences;
}

/**
 * Optimize the relative pose
*/
cv::Mat Optimizer::OptimizeRelativePose(std::vector<cv::Point2f> prev, std::vector<cv::Point2f> curr, cv::Mat RT_SVD, ConfigLoader config_loader)
{

    // Define Solver
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
    typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // Initialize Solver
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // Define a robust kernel
    g2o::RobustKernel *robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");

    // Fix first vertex
    g2o::VertexSE2 *v_origin = new g2o::VertexSE2();
    v_origin->setId(0);
    v_origin->setFixed(true);
    g2o::SE2 pose(0.0, 0.0, 0.0);
    v_origin->setEstimate(pose); // Set to Tcw and fix it
    optimizer.addVertex(v_origin);

    // Second vertex
    g2o::VertexSE2 *v = new g2o::VertexSE2();
    v->setId(1); // Start from index 1, index 0 is fixed
    double tangent = RT_SVD.at<float>(1, 0) / RT_SVD.at<float>(0, 0);
    double angle_yaw = atan2(RT_SVD.at<float>(1, 0), RT_SVD.at<float>(0, 0));
    g2o::SE2 TransRT(RT_SVD.at<float>(0, 2), RT_SVD.at<float>(1, 2), angle_yaw);
    v->setEstimate(TransRT); // Set to Tcw
    optimizer.addVertex(v);

    // Odometry constraint
    g2o::EdgeSE2 *edge = new g2o::EdgeSE2();
    edge->vertices()[0] = optimizer.vertex(0);
    edge->vertices()[1] = optimizer.vertex(1);
    Eigen::Matrix<double, 3, 3> pose_information = Eigen::Matrix<double, 3, 3>::Identity();
    pose_information(0, 0) = pose_information(1, 1) = config_loader.poseInformationXY;
    pose_information(2, 2) = config_loader.poseInformationYaw;
    edge->setInformation(pose_information);
    edge->setMeasurement(TransRT);
    optimizer.addEdge(edge);

    // Add the landmark observations
    std::vector<g2o::EdgeSE2PointXY *> landmarkObservations;
    for (size_t i = 0; i < curr.size(); i++)
    {
        int landmark_vertex_id = 2 + i;
        g2o::VertexPointXY *landmark = new g2o::VertexPointXY;
        landmark->setId(landmark_vertex_id);
        cv::Point2d landmark_xy = prev[i];
        landmark_xy = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth, config_loader.cartImageScale, landmark_xy);
        Eigen::Vector2d landmark_Tcw(landmark_xy.x, landmark_xy.y);
        landmark->setEstimate(landmark_Tcw);
        optimizer.addVertex(landmark);

        // Edge between landmark and frame
        g2o::EdgeSE2PointXY *landmarkObservation = new g2o::EdgeSE2PointXY;
        landmarkObservation->vertices()[0] = optimizer.vertex(1);
        landmarkObservation->vertices()[1] = optimizer.vertex(landmark_vertex_id);
        cv::Point2d obs_xy = curr[i];
        // Convert to image point to local frame coordinate
        obs_xy = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth, config_loader.cartImageScale, obs_xy);
        Eigen::Vector2d obs(obs_xy.x, obs_xy.y);
        landmarkObservation->setMeasurement(obs);
        Eigen::Matrix2d information = Eigen::Matrix<double, 2, 2>::Identity();
        information(0, 0) = information(1, 1) = config_loader.landmarkInformation;
        landmarkObservation->setInformation(information);
        landmarkObservation->setRobustKernel(robustKernel); // Set the robust kernel
        optimizer.addEdge(landmarkObservation);
        landmarkObservations.push_back(landmarkObservation);
    }

    // Optimize
    optimizer.initializeOptimization();
    optimizer.optimize(config_loader.optimizeStep); // Steps

    // Recover RT
    g2o::VertexSE2 *vSE2_recov = static_cast<g2o::VertexSE2 *>(optimizer.vertex(1));
    g2o::SE2 SE2_recov = vSE2_recov->estimate();
    Eigen::Vector3d rt_vec = SE2_recov.toVector();
    cv::Mat RT = toCvMat(SE2_recov);

    // Clear the optimizer
    optimizer.clear();

    return RT;
}

void Optimizer::OptimizeTracking(Frame *pCurrentFrame,
                                 std::vector<Frame *> vpWindowFrames,
                                 std::vector<cv::Mat> vRT,
                                 const ConfigLoader &config_loader)
{
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "OptimizeTracking Frame: " << pCurrentFrame->mnId << std::endl;
    int totalEdge = 0;

    // Define Solver
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
    typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // Initialize Solver
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    // g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(
    //     g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    long unsigned int maxKFid = 0;

    // Define a robust kernel
    g2o::RobustKernel *robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");

    // Collect all Local Features seen in Local Window
    std::list<Feature *> lpWindowFeatures;
    for (std::vector<Frame *>::iterator lit = vpWindowFrames.begin(); lit != vpWindowFrames.end(); lit++)
    {
        std::vector<Feature *> vpFTs = (*lit)->mvFeatures;
        for (std::vector<Feature *>::iterator vit = vpFTs.begin(); vit != vpFTs.end(); vit++)
        {
            Feature *pFT = *vit;
            if (pFT)
                if (pFT->mnBALocalForFrame != pCurrentFrame->mnId)
                {
                    lpWindowFeatures.push_back(pFT);
                    pFT->mnBALocalForFrame = pCurrentFrame->mnId;
                }
        }
    }

    // map of frame index to frame pointer
    std::map<long unsigned int, Frame *> mapIndex2FramePointer;

    // Set Local Frame vertices
    for (std::vector<Frame *>::iterator vit = vpWindowFrames.begin(), vend = vpWindowFrames.end(); vit != vend; vit++)
    {
        Frame *pFi = *vit;
        mapIndex2FramePointer[pFi->mnId] = pFi;

        cv::Mat Tcw = pFi->GetPose();
        double tangent = Tcw.at<float>(1, 0) / Tcw.at<float>(0, 0);
        double angle_yaw = atan2(Tcw.at<float>(1, 0), Tcw.at<float>(0, 0));
        g2o::SE2 pose(Tcw.at<float>(0, 2), Tcw.at<float>(1, 2), angle_yaw);
        g2o::VertexSE2 *vSE2 = new g2o::VertexSE2();
        vSE2->setEstimate(pose);
        vSE2->setId(pFi->mnId);
        vSE2->setFixed(pFi->mnId == 1); // First frame idx == 1 got fixed
        // std::cout << "mnId: " << pFi->mnId << " SE2_origin: " << std::endl
        //           << pose.toVector() << std::endl;

        optimizer.addVertex(vSE2);
        if (pFi->mnId > maxKFid)
            maxKFid = pFi->mnId;
    }

    // Set Frames edge contraints
    Frame *pFramePrev;
    Frame *pFrameNext;
    int i = 0;
    for (std::vector<Frame *>::iterator vit = vpWindowFrames.begin(), vend = vpWindowFrames.end(); vit != vend; vit++)
    {
        if (vit == vpWindowFrames.begin())
        {
            pFramePrev = *vit;
        }
        else
        {
            pFrameNext = *vit;
            cv::Mat RT = vRT[i];
            // std::cout << "RT: " << std::endl
            //   << RT << std::endl;
            g2o::EdgeSE2 *edgeNew = new g2o::EdgeSE2();
            edgeNew->vertices()[0] = optimizer.vertex(pFramePrev->mnId);
            edgeNew->vertices()[1] = optimizer.vertex(pFrameNext->mnId);
            double angle_yaw = atan2(RT.at<float>(1, 0), RT.at<float>(0, 0));
            g2o::SE2 Scm(RT.at<float>(0, 2), RT.at<float>(1, 2), angle_yaw);
            edgeNew->setMeasurement(Scm);
            // Information matrix for loop closure
            Eigen::Matrix<double, 3, 3> information = Eigen::Matrix<double, 3, 3>::Identity();
            information(0, 0) = information(1, 1) = config_loader.poseInformationXY;
            information(2, 2) = config_loader.poseInformationYaw;
            edgeNew->setInformation(information);
            optimizer.addEdge(edgeNew);

            // Update variable
            i++;
            pFramePrev = pFrameNext;
            totalEdge++;
        }
    }

    // Set Local Feature vertices
    std::vector<g2o::EdgeSE2PointXY *> vEdges;
    for (std::list<Feature *>::iterator lit = lpWindowFeatures.begin(), lend = lpWindowFeatures.end(); lit != lend; lit++)
    {
        Feature *pFT = *lit;
        g2o::VertexPointXY *vertexFeature = new g2o::VertexPointXY;
        int id = pFT->mnId + maxKFid + 1;
        vertexFeature->setId(id);
        cv::Mat WorldPos = pFT->mWorldPos;
        Eigen::Vector2d EigenWorldPos(WorldPos.at<float>(0, 0), WorldPos.at<float>(1, 0));
        vertexFeature->setEstimate(EigenWorldPos);
        optimizer.addVertex(vertexFeature);
        const std::map<long unsigned int, size_t> observations = pFT->mObservations;
        // std::cout << "pFT->WorldPos_orig: " << std::endl
        //   << pFT->mWorldPos << std::endl;
        // std::cout << "Feature " << pFT->mnId << std::endl;

        // Set edges
        for (std::map<long unsigned int, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {

            long unsigned int frameIdx = mit->first;
            if (!mapIndex2FramePointer.count(frameIdx)) // if the observation is not in the local window, skip
                continue;

            Frame *pFi = mapIndex2FramePointer[frameIdx];
            size_t nKeyPoint = mit->second;

            g2o::EdgeSE2PointXY *e = new g2o::EdgeSE2PointXY;
            e->vertices()[0] = optimizer.vertex(pFi->mnId);
            e->vertices()[1] = optimizer.vertex(id);

            cv::Point2f image_point = pFi->mvFeatureCoordinates[nKeyPoint];

            cv::Point2f obs_xy = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth,
                                                       config_loader.cartImageScale, image_point);
            Eigen::Vector2d measurement(obs_xy.x, obs_xy.y);
            e->setMeasurement(measurement);

            // Set information
            Eigen::Matrix2d information = Eigen::Matrix<double, 2, 2>::Identity();

            // Encode scanning motion prior here into information
            double factor = 1; // initialization
            // double theta = atan2(obs_xy.y, obs_xy.x);
            // if (theta <= M_PI /2 && theta > 0)
            // {
            //     factor = (2*M_PI  - (M_PI /2 - theta)) / (2*M_PI );
            // }else if (theta > M_PI / 2)
            // {
            //     factor = (theta - M_PI /2) / (2*M_PI );
            // }else
            // {
            //     factor = (M_PI /2 - theta) / (2*M_PI );
            // }

            information(0, 0) = information(1, 1) = config_loader.landmarkInformation * factor;
            e->setInformation(information);

            e->setRobustKernel(robustKernel);

            optimizer.addEdge(e);
            vEdges.push_back(e);
            totalEdge++;
        }
    }

    auto start1 = std::chrono::high_resolution_clock::now();

    // std::string g2o_result_before = "/home/hong/Documents/g2o/radar_localization/" + std::to_string(pCurrentFrame->mnId) + "_localWindow_pose_graph_before.g2o";
    // std::string g2o_result_after = "/home/hong/Documents/g2o/radar_localization/" + std::to_string(pCurrentFrame->mnId) + "_localWindow_pose_graph_after.g2o";
    // optimizer.save(g2o_result_before.c_str());
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    // optimizer.save(g2o_result_after.c_str());
    // auto stop1 = std::chrono::high_resolution_clock::now();
    // auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start1);
    // std::cout << "Total number of edges: " << totalEdge << std::endl;
    // std::cout << "optimization time: " << duration1.count() << " ms." <<std::endl;

    /* Recover optimized data */

    // Frames
    for (std::vector<Frame *>::iterator lit = vpWindowFrames.begin(); lit != vpWindowFrames.end(); lit++)
    {
        Frame *pFi = *lit;
        g2o::VertexSE2 *vSE2_recov = static_cast<g2o::VertexSE2 *>(optimizer.vertex(pFi->mnId));
        g2o::SE2 SE2_recov = vSE2_recov->estimate();
        // std::cout << "mnId: " << pFi->mnId << " SE2_recov: " << std::endl
        //           << SE2_recov.toVector() << std::endl;
        cv::Mat optimized_Tcw = toCvMat(SE2_recov);
        pFi->SetPose(optimized_Tcw);
    }

    // Features
    for (std::list<Feature *>::iterator lit = lpWindowFeatures.begin(); lit != lpWindowFeatures.end(); lit++)
    {
        Feature *pFT = *lit;
        g2o::VertexPointXY *vPoint = static_cast<g2o::VertexPointXY *>(optimizer.vertex(pFT->mnId + maxKFid + 1));
        Eigen::Vector2d EigenWorldPos = vPoint->estimate();
        pFT->SetWorldPos(toCvMat(EigenWorldPos));
        // std::cout << "pFT->WorldPos_recov: " << std::endl
        //   << pFT->mWorldPos << std::endl;
    }
    // std::cout << "3" << std::endl;
    std::cout << "Finish tracking optimization." << std::endl;
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Tracking optimization time: " << duration.count() << " ms." << std::endl;
}

void Optimizer::OptimizeTrackPoints(Frame *pCurrentFrame,
                                    Frame *pLastFrame,
                                    std::vector<Frame *> vpWindowFrames,
                                    const ConfigLoader &config_loader)
{
    auto start = std::chrono::high_resolution_clock::now();

    int totalEdge = 0;
    double factorT0 = 0.5; // The factor of T(t=0)
    const int iterations = config_loader.iterations;
    std::string frame_id = std::to_string(pCurrentFrame->mnId);

    // Define Solver
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
    typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // Initialize Solver
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // Define a robust kernel
    g2o::RobustKernel *robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");

    // map of frame index to frame pointer
    std::map<long unsigned int, Frame *> mapIndex2FramePointer;

    // Push last frame into local window first
    vpWindowFrames.push_back(pLastFrame);

    // Get Local Frame Pointer
    for (std::vector<Frame *>::iterator vit = vpWindowFrames.begin(), vend = vpWindowFrames.end(); vit != vend; vit++)
    {
        // Frame vertex
        Frame *pFi = *vit;
        mapIndex2FramePointer[pFi->mnId] = pFi;
    }

    // Collect Features seen in current frame
    std::vector<Feature *> vpFeature = pCurrentFrame->mvFeatures;

    // Set Local Feature vertices and edges
    std::vector<g2o::EdgePointXYOnly *> vpEdgePointXYOnly;
    for (size_t i = 0; i < vpFeature.size(); i++)
    {
        Feature *pFT = vpFeature[i];

        g2o::VertexPointXY *vertexFeature = new g2o::VertexPointXY;
        vertexFeature->setId(pFT->mnId);
        cv::Mat WorldPos = pFT->mWorldPos;
        Eigen::Vector2d EigenWorldPos(WorldPos.at<float>(0, 0), WorldPos.at<float>(1, 0));
        vertexFeature->setEstimate(EigenWorldPos);
        optimizer.addVertex(vertexFeature);
        const std::map<long unsigned int, size_t> observations = pFT->mObservations;

        // Set edges
        for (std::map<long unsigned int, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {

            long unsigned int frameIdx = mit->first;
            if (!mapIndex2FramePointer.count(frameIdx)) // if the observation is not in the local window, skip
                continue;

            Frame *pFi = mapIndex2FramePointer[frameIdx];
            size_t nKeyPoint = mit->second;

            // Edge
            g2o::EdgePointXYOnly *e = new g2o::EdgePointXYOnly;
            e->vertices()[0] = optimizer.vertex(pFT->mnId); // Feature

            // Set Velocity
            e->setVelocity(pFi->GetVelocity());

            // Set SE2
            cv::Mat Tcw = pFi->GetPose();
            double angle_yaw = atan2(Tcw.at<float>(1, 0), Tcw.at<float>(0, 0));
            g2o::SE2 pose(Tcw.at<float>(0, 2), Tcw.at<float>(1, 2), angle_yaw);

            // Set measurement
            cv::Point2f image_point = pFi->mvFeatureCoordinates[nKeyPoint];
            cv::Point2f obs_xy = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth,
                                                       config_loader.cartImageScale, image_point);
            Eigen::Vector2d measurement(obs_xy.x, obs_xy.y);
            e->setMeasurement(measurement);

            // Compute delta time t between t=0 and the time when landmark is observed
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
            double frameTimeDiff = pFi->mTimestamp - pFi->mPreviousTimestamp;
            double deltaT = frameTimeDiff * (factorT - factorT0);
            e->setDeltaT(deltaT);

            // Set information
            Eigen::Matrix2d information = Eigen::Matrix<double, 2, 2>::Identity();
            information(0, 0) = information(1, 1) = config_loader.landmarkInformation;
            e->setInformation(information);

            // Set Robust Kernel
            e->setRobustKernel(robustKernel);

            optimizer.addEdge(e);
            vpEdgePointXYOnly.push_back(e);
            totalEdge++;
        }
    }

    /* Optimize! */
    std::string g2o_result_before = "/home/hong/Desktop/PoseError/trackPoints_g2o/" +
                                     std::string(6 - frame_id.length(), '0').append(frame_id) +
                                      "_trackPoints_before.g2o";
    std::string g2o_result_after = "/home/hong/Desktop/PoseError/trackPoints_g2o/" + 
                                    std::string(6 - frame_id.length(), '0').append(frame_id) +
                                     "_trackPoints_after.g2o";
    optimizer.save(g2o_result_before.c_str());
    optimizer.initializeOptimization(0);
    optimizer.optimize(10);
    optimizer.save(g2o_result_after.c_str());

    /* Recover optimized data */

    // Features
    for (size_t i = 0; i < vpFeature.size(); i++)
    {
        Feature *pFT = vpFeature[i];
        g2o::VertexPointXY *vPoint = static_cast<g2o::VertexPointXY *>(optimizer.vertex(pFT->mnId));
        Eigen::Vector2d EigenWorldPos = vPoint->estimate();
        pFT->SetWorldPos(toCvMat(EigenWorldPos));
    }

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "OptimizeTrackPoints optimization time: " << duration.count() << " ms." << std::endl;
}

void Optimizer::OptimizeTrackingMotionPrior(Frame *pCurrentFrame,
                                            Frame *pLastFrame,
                                            std::vector<Frame *> vpWindowFrames,
                                            const ConfigLoader &config_loader)
{
    auto start = std::chrono::high_resolution_clock::now();
    // std::cout << "OptimizeTracking Frame: " << pCurrentFrame->mnId << std::endl;
    int totalEdge = 0;
    std::string frame_id = std::to_string(pCurrentFrame->mnId);

    // Parameters
    double factorT0 = 0.5; // The factor of T(t=0)
    const int iterations = config_loader.iterations;
    const int optimizations = 1;

    // Define Solver
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
    typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

    // Initialize Solver
    auto linearSolver = g2o::make_unique<SlamLinearSolver>();
    linearSolver->setBlockOrdering(false);

    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    // g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(
    //     g2o::make_unique<SlamBlockSolver>(std::move(linearSolver)));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // Define a robust kernel
    g2o::RobustKernel *robustKernel = g2o::RobustKernelFactory::instance()->construct("Cauchy");
    // g2o::RobustKernelHuber *robustKernel = new g2o::RobustKernelHuber;
    // const float deltaHuber = sqrt(5.991);
    // robustKernel->setDelta(deltaHuber);

    long unsigned int maxKFid = 0;

    // map of frame index to frame pointer
    std::map<long unsigned int, Frame *> mapIndex2FramePointer;

    // Push last frame into the local windows first
    vpWindowFrames.push_back(pLastFrame);

    // Set for the vertex number
    std::unordered_set<size_t> sVertexID;

    // Set Local Frame vertices and velocity vertices
    for (std::vector<Frame *>::iterator vit = vpWindowFrames.begin(), vend = vpWindowFrames.end(); vit != vend; vit++)
    {
        // Frame vertex
        Frame *pFi = *vit;
        size_t FrameVertexId = pFi->mnId * 2 - 1;
        mapIndex2FramePointer[pFi->mnId] = pFi;

        cv::Mat Tcw = pFi->GetPose();
        double tangent = Tcw.at<float>(1, 0) / Tcw.at<float>(0, 0);
        double angle_yaw = atan2(Tcw.at<float>(1, 0), Tcw.at<float>(0, 0));
        g2o::SE2 pose(Tcw.at<float>(0, 2), Tcw.at<float>(1, 2), angle_yaw);
        g2o::VertexSE2 *vSE2 = new g2o::VertexSE2();
        vSE2->setEstimate(pose);
        vSE2->setId(FrameVertexId);
        vSE2->setFixed(FrameVertexId == 1); // First frame idx == 1 got fixed
        // vSE2->setFixed(FrameVertexId != pCurrentFrame->mnId * 2 - 1); // Fix all other frames except for current frame
        // vSE2->setFixed(true); // Fix all poses
        // std::cout << "mnId: " << pFi->mnId << " SE2_origin: " << std::endl
        //           << pose.toVector() << std::endl;

        // Velocity vertex
        size_t VelocityVertexId = pFi->mnId * 2;
        g2o::VertexPointXYZ *vVelocity = new g2o::VertexPointXYZ();
        // g2o::VertexPointXY *vVelocity = new g2o::VertexPointXY();

        Eigen::Matrix<double, 3, 1> velocity = pFi->GetVelocity();
        vVelocity->setEstimate(velocity);
        // vVelocity->setEstimate(velocity.head<2>());
        vVelocity->setId(VelocityVertexId);
        vVelocity->setFixed(VelocityVertexId == 2); // First velocity idx == 2 got fixed
        // vVelocity->setFixed(VelocityVertexId != pCurrentFrame->mnId * 2); // Fix all other velocity except for current frame

        if (!sVertexID.count(VelocityVertexId)) 
        {
            // std::cout << "Element not found" << std::endl;
            sVertexID.insert(VelocityVertexId);
            // Add pose vertex
            optimizer.addVertex(vSE2);
            // Add velocity vertex
            optimizer.addVertex(vVelocity);
        }
  
        // // Add pose vertex
        // optimizer.addVertex(vSE2);
        // // Add velocity vertex
        // optimizer.addVertex(vVelocity);

        if (VelocityVertexId > maxKFid)
            maxKFid = VelocityVertexId;
    }

    // // Collect all Local Features seen in Local Window
    // std::list<Feature *> lpWindowFeatures;
    // for (std::vector<Frame *>::iterator lit = vpWindowFrames.begin(); lit != vpWindowFrames.end(); lit++)
    // {
    //     std::vector<Feature *> vpFTs = (*lit)->mvFeatures;
    //     for (std::vector<Feature *>::iterator vit = vpFTs.begin(); vit != vpFTs.end(); vit++)
    //     {
    //         Feature *pFT = *vit;
    //         if (pFT)
    //             if (pFT->mnBALocalForFrame != pCurrentFrame->mnId)
    //             {
    //                 lpWindowFeatures.push_back(pFT);
    //                 pFT->mnBALocalForFrame = pCurrentFrame->mnId;
    //             }
    //     }
    // }

    // Collect Features seen in current frame
    std::list<Feature *> lpWindowFeatures(pCurrentFrame->mvFeatures.begin(), pCurrentFrame->mvFeatures.end());

    // Edge Observation stream
    // std::ofstream EdgeObsStream;
    // std::string edge_obs_file_name = "/home/hong/Desktop/PoseError/EdgeError/" +
    //                                  std::string(6 - frame_id.length(), '0').append(frame_id) + "_LocalObservation.csv";
    // EdgeObsStream.open(edge_obs_file_name);
    // EdgeObsStream << "PointID,Pw.x,Pw.y,FrameID,Local.x,Local.y,\n";

    // Set Local Feature vertices and edges
    std::vector<g2o::EdgeSE2VelocityPointFixed *> vpEdgeSE2VelocityPointFixed;
    std::vector<Feature *> vpFeature;
    for (std::list<Feature *>::iterator lit = lpWindowFeatures.begin(), lend = lpWindowFeatures.end(); lit != lend; lit++)
    {
        Feature *pFT = *lit;
        // g2o::VertexPointXY *vertexFeature = new g2o::VertexPointXY;
        // int id = pFT->mnId + maxKFid + 1;
        // vertexFeature->setId(id);
        cv::Mat WorldPos = pFT->mWorldPos;
        Eigen::Vector2d EigenWorldPos(WorldPos.at<float>(0, 0), WorldPos.at<float>(1, 0));
        // vertexFeature->setEstimate(EigenWorldPos);
        // optimizer.addVertex(vertexFeature);
        // vertexFeature->setFixed(true);
        const std::map<long unsigned int, size_t> observations = pFT->mObservations;

        // Set edges
        for (std::map<long unsigned int, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
        {

            long unsigned int frameIdx = mit->first;
            if (!mapIndex2FramePointer.count(frameIdx)) // if the observation is not in the local window, skip
                continue;

            Frame *pFi = mapIndex2FramePointer[frameIdx];
            size_t nKeyPoint = mit->second;

            // Edge
            g2o::EdgeSE2VelocityPointFixed *e = new g2o::EdgeSE2VelocityPointFixed;
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pFi->mnId * 2 - 1))); // Pose
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pFi->mnId * 2)));     // Velocity

            // Fix Z velocity
            // Eigen::Vector3d velocity = pFi->GetVelocity();
            // e->setVelocityZ(velocity(2));

            // Log
            e->log(pCurrentFrame->mnId, pFT->mnId, pFi->mnId);

            // Set Point world position
            e->setPointWorldPos(EigenWorldPos);

            // Set measurement
            cv::Point2f image_point = pFi->mvFeatureCoordinates[nKeyPoint];
            cv::Point2f obs_xy = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth,
                                                       config_loader.cartImageScale, image_point);
            Eigen::Vector2d measurement(obs_xy.x, obs_xy.y);
            e->setMeasurement(measurement);

            // EdgeObsStream << pFT->mnId << "," << EigenWorldPos(0) << "," << EigenWorldPos(1) << ","
            //               << pFi->mnId << "," << obs_xy.x << "," << obs_xy.y << "\n";

            // Compute delta time t between t=0 and the time when landmark is observed
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
            double frameTimeDiff = pFi->mTimestamp - pFi->mPreviousTimestamp;
            double deltaT = frameTimeDiff * (factorT - factorT0);
            e->setDeltaT(deltaT);
            // std::cout << "frameTimeDiff: " << frameTimeDiff << std::endl;

            // Set information
            Eigen::Matrix2d information = Eigen::Matrix<double, 2, 2>::Identity();
            information(0, 0) = information(1, 1) = config_loader.landmarkInformation;
            e->setInformation(information);

            // Set Robust Kernel
            e->setRobustKernel(robustKernel);

            optimizer.addEdge(e);
            vpEdgeSE2VelocityPointFixed.push_back(e);
            vpFeature.push_back(pFT);
            totalEdge++;
        }
    }

    /* Regularization */

    /**
         * Edge for Velocity Prior Term
         * Correlate velocity with pose.
         */
    g2o::EdgeVelocitySE2 *edgeVelPose = new g2o::EdgeVelocitySE2;
    edgeVelPose->vertices()[0] = optimizer.vertex(pCurrentFrame->mnId * 2);
    edgeVelPose->vertices()[1] = optimizer.vertex(pCurrentFrame->mnId * 2 - 1);
    int numFrames = vpWindowFrames.size();
    // std::cout << "Number of frames in the window: " << numFrames << std::endl;
    // Frame *pFi = vpWindowFrames[numFrames - 2]; // The closest frame in the window
    // std::cout << "pFi->mnId: " << pFi->mnId * 2 - 1 << std::endl;
    // edgeVelPose->vertices()[2] = optimizer.vertex(pFi->mnId * 2 - 1);
    // edgeVelPose->vj = static_cast<g2o::VertexSE2 *>(optimizer.vertex(pFi->mnId * 2 - 1));
    // double timeDiff = pCurrentFrame->mTimestamp - pFi->mTimestamp;

    // std::cout << "pLastFrame->mnId: " << pLastFrame->mnId  << std::endl;
    edgeVelPose->vj = static_cast<g2o::VertexSE2 *>(optimizer.vertex(pLastFrame->mnId * 2 - 1));
    double timeDiff = pCurrentFrame->mTimestamp - pLastFrame->mTimestamp;
    // std::cout << "timeDiff: " << timeDiff << std::endl;
    edgeVelPose->setDeltaT(timeDiff);

    // Set log
    edgeVelPose->log(pCurrentFrame->mnId);

    // Set information
    Eigen::Matrix3d information = Eigen::Matrix<double, 3, 3>::Identity();
    // Eigen::Matrix2d information = Eigen::Matrix<double, 2, 2>::Identity();
    // Eigen::Matrix<double, 1, 1> information ;

    double informationX = config_loader.landmarkInformation * config_loader.motionPriorFactorX;
    double informationY = config_loader.landmarkInformation * config_loader.motionPriorFactorY;
    double informationYaw = config_loader.landmarkInformation * config_loader.motionPriorFactorYaw;
    information(0, 0) = informationX;
    information(1, 1) = informationY;
    information(2, 2) = informationYaw;

    edgeVelPose->setInformation(information);
    optimizer.addEdge(edgeVelPose);

    // /**
    //  * Edge for smoothing velocity
    //  * using previous velocity.
    //  * Or using the prior velocity computed from relative motion.
    //  *
    //  */
    // g2o::EdgeVelocityOnly *e = new g2o::EdgeVelocityOnly;
    // e->vertices()[0] = optimizer.vertex(pCurrentFrame->mnId * 2);
    // // Set information
    // Eigen::Matrix3d info = Eigen::Matrix<double, 3, 3>::Identity();
    // double infoX = config_loader.landmarkInformation * config_loader.motionPriorFactorX;
    // double infoY = config_loader.landmarkInformation * config_loader.motionPriorFactorY;
    // double infoYaw = config_loader.landmarkInformation * config_loader.motionPriorFactorYaw;
    // info(0, 0) = infoX;
    // info(1, 1) = infoY;
    // info(2, 2) = infoYaw;
    // e->setInformation(info);
    // Eigen::Matrix<double, 3, 1> meas = pCurrentFrame->GetVelocity();
    // meas(1) = 0.0;
    // e->setMeasurement(meas);
    // optimizer.addEdge(e);

    /* Optimize! */

    // auto start1 = std::chrono::high_resolution_clock::now();
    // std::string g2o_result_before = config_loader.dataset_path + "g2o/" +
    //                                  std::string(6 - frame_id.length(), '0').append(frame_id) +
    //                                   "_motionPrior_before.g2o";
    // std::string g2o_result_after =  config_loader.dataset_path + "g2o/" + 
    //                                 std::string(6 - frame_id.length(), '0').append(frame_id) +
    //                                  "_motionPrior_after.g2o";

    // std::string g2o_result_before = "/home/hong/Desktop/PoseError/trackPoints_g2o/" +
    //                                  std::string(6 - frame_id.length(), '0').append(frame_id) +
    //                                   "_trackPoints_before.g2o";
    // std::string g2o_result_after = "/home/hong/Desktop/PoseError/trackPoints_g2o/" + 
    //                                 std::string(6 - frame_id.length(), '0').append(frame_id) +
    //                                  "_trackPoints_after.g2o";

    // optimizer.save(g2o_result_before.c_str());

    // Edge errors stream
    // std::ofstream EdgeErrorStream;
    // std::string edge_error_file_name = "/home/hong/Desktop/PoseError/EdgeError/" +
    //                                    std::string(6 - frame_id.length(), '0').append(frame_id) + "_EdgeError.csv";
    // EdgeErrorStream.open(edge_error_file_name);
    // EdgeErrorStream << "iteration,Pw.ID,Pw.x,Pw.y,T0w.x,T0w.y,T0w.theta,chi2Error\n";

    // We perform k optimizations, after first optimization we classify observation as inlier/outlier
    const float chi2 = 5.991;
    for (size_t it = 0; it < optimizations; it++)
    {

        optimizer.initializeOptimization(0);
        optimizer.optimize(iterations);

        // Features culling based on the chi2 error
        for (int i = 0; i < vpEdgeSE2VelocityPointFixed.size(); i++)
        {
            g2o::EdgeSE2VelocityPointFixed *e = vpEdgeSE2VelocityPointFixed[i];
            g2o::VertexSE2 *v = static_cast<g2o::VertexSE2 *>(e->vertices()[0]);

            Feature *pFT = vpFeature[i];
            cv::Mat WorldPos = pFT->mWorldPos;

            // EdgeErrorStream << it << "," << pFT->mnId << "," << WorldPos.at<float>(0, 0) << "," << WorldPos.at<float>(1, 0) << ","
            //                 << v->estimate().translation()[0] << "," << v->estimate().translation()[1] << ","
            //                 << v->estimate().rotation().angle() << "," << e->chi2() << "\n";

            // if (e->chi2() > chi2)
            //     e->setLevel(1);
        }
    }

    // optimizer.save(g2o_result_after.c_str());
    // auto stop1 = std::chrono::high_resolution_clock::now();
    // auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop1 - start1);
    // std::cout << "optimization time: " << duration1.count() << " ms." << std::endl;

    /* Recover optimized data */

    // Optimized variables stream
    // std::ofstream variablesStream;
    // std::string optimized_variables_file_name = "/home/hong/Desktop/PoseError/EdgeError/" +
    //                                             std::string(6 - frame_id.length(), '0').append(frame_id) +                "_OptimizedVariables.csv";
    // variablesStream.open(optimized_variables_file_name);
    // variablesStream << "ID,T0w.x,T0w.y,T0w.theta,Velocity.x,Velocity.y,Velocity.z\n";

    // Frames and Velocity
    for (std::vector<Frame *>::iterator lit = vpWindowFrames.begin(); lit != vpWindowFrames.end(); lit++)
    {
        // Frame
        Frame *pFi = *lit;
        g2o::VertexSE2 *vSE2_recov = static_cast<g2o::VertexSE2 *>(optimizer.vertex(pFi->mnId * 2 - 1));
        g2o::SE2 SE2_recov = vSE2_recov->estimate();
        // std::cout << "Frame mnId: " << pFi->mnId << " SE2_recov: " << std::endl
        Eigen::Vector3d poseVec = SE2_recov.toVector();
        cv::Mat optimized_Tcw = toCvMat(SE2_recov);
        pFi->SetPose(optimized_Tcw);

        // Velocity
        g2o::VertexPointXYZ *vVelocity_recov = static_cast<g2o::VertexPointXYZ *>(optimizer.vertex(pFi->mnId * 2));
        Eigen::Matrix<double, 3, 1> velocity = vVelocity_recov->estimate();

        // g2o::VertexPointXY *vVelocity_recov = static_cast<g2o::VertexPointXY *>(optimizer.vertex(pFi->mnId * 2));
        // Eigen::Matrix<double, 2, 1> velocityXY = vVelocity_recov->estimate();
        // Eigen::Matrix<double, 3, 1> velocity = pFi->GetVelocity();
        // velocity.head(2) = velocityXY;

        // std::cout << "Velocity: " << std::endl
        //   << velocity << std::endl
        //   << std::endl;
        pFi->SetVelocity(velocity);

        // variablesStream << pFi->mnId << "," << poseVec(0) << "," << poseVec(1) << "," << poseVec(2) << ","
        //                 << velocity(0) << "," << velocity(1) << "," << velocity(2) << "\n";
    }

    // std::cout << "Total number of edges: " << totalEdge << std::endl;
    // std::cout << "Finish tracking optimization." << std::endl;
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    // std::cout << "OptimizeTrackingMotionPrior optimization time: " << duration.count() << " ms." << std::endl;
}


