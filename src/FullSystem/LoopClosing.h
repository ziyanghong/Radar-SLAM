#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

// #include <feature/Detector.h>
// #include <feature/ShapeContext.h>
// #include <feature/BetaGrid.h>
// #include <feature/RangeDetector.h>
// #include <feature/CurvatureDetector.h>
// #include <feature/NormalBlobDetector.h>
// #include <feature/NormalEdgeDetector.h>
// #include <feature/RansacFeatureSetMatcher.h>
// #include <feature/RansacMultiFeatureSetMatcher.h>
// #include <sensorstream/CarmenLog.h>
// #include <sensorstream/LogSensorStream.h>
// #include <sensorstream/SensorStream.h>
// #include <utils/SimpleMinMaxPeakFinder.h>
// #include <utils/HistogramDistances.h>

// #include <cairo.h>
// #include <cairo-pdf.h>
// #include <cairo-svg.h>


#include <iostream>
#include <string>
#include <string.h>
#include <sstream>
#include <utility>
#include <sys/stat.h>
#include <sys/types.h>

#include "thread"
#include "mutex"


// Loader
#include "../util/Loader.h"
// Frame
#include "Frame.h"
// KeyFrame
#include"KeyFrame.h"
// MapPoint
#include"MapPoint.h"
// Map
#include "Map.h"
// CFAR
#include "cfar.h"
// Polar to Cartesian
#include "../util/polarToCartesian.h"
// Algorithms
#include"Algorithms.h"
// Matcher
#include "Matcher.h"
// Converter
#include "Converter.h"
// Viewer
#include "Viewer.h"
// KDTree
#include "KDTree.h"
// Tracker
#include "Tracking.h"



class LocalMapping;
class KeyFrame;
class Frame;
class Tracking;
class Viewer;

class LoopClosing
{
    public:
        LoopClosing(Map* pMap,  const ConfigLoader config_loader);

        void SetTracker(Tracking* pTracker);

        void SetLocalMapper(LocalMapping* pLocalMapper);

        void SetViewer(Viewer* pViewer); 

        // Main function 
        void Run();

        void InsertKeyFrame(KeyFrame *pKF);

        // This function will run in a separate thread
        void RunGlobalBundleAdjustment(unsigned long nLoopKF);

        // ---------------------------------------------------Variables--------------------------------------------------
        int mnLoopCandidate;

    protected:
        bool CheckNewKeyFrames();

        bool DetectLoop();
        
        bool ComputeSE2();

        std::vector<KeyFrame*> SamplePotentialLoops();

        void CorrectLoop();

        void CorrectLocalWindow();

        void SetFinish();

        ConfigLoader mConfigLoader;

        bool mbFinished;
        std::mutex mMutexFinish;

        Map* mpMap;
        Tracking* mpTracker;

        LocalMapping *mpLocalMapper;

        Viewer* mpViewer;

        std::list<KeyFrame*> mlpLoopKeyFrameQueue;
        std::mutex mMutexLoopQueue;


        // Loop detector variables
        KeyFrame* mpCurrentKF;
        KeyFrame* mpMatchedKF;
        std::vector<KeyFrame*> mvpCurrentConnectedKFs;
        cv::Mat mTcm; // The corrected pose of current frame
        cv::Mat mLoopRT; // Relative transformation between current keyframe to loop
        long unsigned int mLastLoopKFid;
        bool mbReverseLoop;

        // Loop relative poses result file
        std::ofstream ofsLoop;


        // Variables related to Global Bundle Adjustment
        bool mbFinishedGBA;
        std::mutex mMutexGBA;
        std::thread* mpThreadGBA;
};





#endif 