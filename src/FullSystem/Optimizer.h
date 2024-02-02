#ifndef OPTIMIZER_H
#define OPTIMIZER_H

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
#include <sys/stat.h>
#include <cstdlib>
#include <time.h>
#include <chrono>
#include <iterator>
#include <mutex>
#include <unordered_set>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

// Opencv header
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/persistence.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/videostab/global_motion.hpp"

// g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include "g2o/solvers/eigen/linear_solver_eigen.h"

#include <g2o/types/slam2d/types_slam2d.h> // vertex type
#include <g2o/types/slam3d/types_slam3d.h>

// Self define g2o edge
#include "EdgeSE2XYOnlyPose.h"
#include "EdgeSelfDefined.h"

// Frame
#include "Frame.h"
// KeyFrame
#include "KeyFrame.h"
// MapPoint
#include "MapPoint.h"
// Map
#include "Map.h"
// CFAR
#include "cfar.h"
// Converter
#include "Converter.h"
// Algorithms
#include "Algorithms.h"
// Loader
#include "../util/Loader.h"
// LocalMapping
#include "LocalMapping.h"
// Feature
#include "Feature.h"


class Optimizer
{

public:
    // ------------------------------------------Constructor and Destructor------------------------------------------
    Optimizer();
    ~Optimizer();

    // -------------------------------------------Optimization Functions--------------------------------------------------
    void static BundleAdjustment(const std::vector<KeyFrame *> &vpKF, const std::vector<MapPoint *> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
                                 const bool bRobust = true, const ConfigLoader *pConfigLoader = NULL);

    void static GlobalBundleAdjustment(Map *pMap, int nIterations = 5, bool *pbStopFlag = NULL,
                                       const unsigned long nLoopKF = 0, const bool bRobust = true, const ConfigLoader *pConfigLoader = NULL);

    void static LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, ConfigLoader config_loader);

    void static OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                       const cv::Mat &RT, const cv::Mat &Tcm, ConfigLoader config_loader);

    int static PoseOptimization(Frame *pFrame, ConfigLoader config_loader);

    cv::Mat static OptimizeRelativePose(std::vector<cv::Point2f> prev,
                                        std::vector<cv::Point2f> curr,
                                        cv::Mat RT_SVD,
                                        ConfigLoader config_loader);

    void static OptimizeTracking(Frame *pCurrentFrame,
                                 std::vector<Frame *> vpWindowFrames,
                                 std::vector<cv::Mat> vRT,
                                 const ConfigLoader &config_loader);

    void static OptimizeTrackingMotionPrior(Frame *pCurrentFrame,
                                            Frame *pLastFrame,
                                            std::vector<Frame *> vpWindowFrames,
                                            const ConfigLoader &config_loader);

    void static OptimizeTrackPoints(Frame *pCurrentFrame,
                                    Frame *pLastFrame,
                                            std::vector<Frame *> vpWindowFrames,
                                            const ConfigLoader &config_loader);


};

#endif