#ifndef SYSTEM_H
#define SYSTEM_H

// Standard library
#include <string>
#include <vector>
#include <numeric>
#include <algorithm>
#include <functional>
#include <array>
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
#include <assert.h> 
#include <thread>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Sparse>

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

// Boost
#include <boost/shared_ptr.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

// Loader
#include "../util/Loader.h"
// Mercator
#include "../util/mercator.h"
// Frame
#include "Frame.h"
// KeyFrame
#include"KeyFrame.h"
// MapPoint
#include"MapPoint.h"
// Map
#include "Map.h"
// Tracking
#include "Tracking.h"
// Loop closing
#include "LoopClosing.h"
// CFAR
#include "cfar.h"
// Polar to Cartesian
#include "../util/polarToCartesian.h"
// Optimizer
#include"Optimizer.h"
// Algorithms
#include"Algorithms.h"
// Viewer
#include "Viewer.h"
// Feature tracker
#include "FeatureTracker.h"

// namespace ROAM
// {



class System {
    

public:
    // Tracker
    Tracking* mpTracker;

    std::vector<Frame*> localFrames; // Local BA map

    // KeyFrame poses result file
    std::ofstream ofsKeyFrame;
    // MapPoint postion result file;
    std::ofstream ofsMapPoint;
 

    // ------------------------------------------Constructor and Destructor------------------------------------------
    System();
    System(ConfigLoader );
    ~System();

    // ------------------------------------------------ Functions ----------------------------------------------------
    void SaveMapPoints();
    void SaveKeyFramePoses();
    void FinalGlobalBA();
    void SaveAllFeaturePoints();

    

private:
    // config 
    ConfigLoader mConfigLoader;

    // Map
    Map* mpMap;

    // Viewer
    Viewer* mpViewer;

    // Local mapper
    LocalMapping* mpLocalMapper; 

    // Loop closer
    LoopClosing* mpLoopCloser;

    FeatureTracker* mpFeatureTracker;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptViewer;
    std::thread* mptLoopCloser;
    
};

// } // namespace ROAM


#endif