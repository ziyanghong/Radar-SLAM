#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Optimizer.h"
#include "Map.h"

// In the journal version, we are not performing local mapping,
// we still keep this part of code to have the system running. 

// namespace ROAM{

class MapPoint;
class KeyFrame;
class Map;
class Tracking;
class LoopClosing;

class LocalMapping{
protected:
 
    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);


    bool CheckFinish();
    void SetFinish();

    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;
    
    Map* mpMap;
    Tracking* mpTracker;
    LoopClosing* mpLoopCloser;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

public:
    LocalMapping();
    ~LocalMapping();
    LocalMapping(Map* pMap, ConfigLoader _config_loader);    
    void Run();
    void SetTracker(Tracking* pTracker);
    void SetLoopCloser(LoopClosing* pLoopCloser);
    void InsertKeyFrame(KeyFrame* pKF);
    // std::list<KeyFrame*> GetNewKeyFrames();

    // Configuration
    ConfigLoader config_loader;

    // Thread Synch
    void RequestStop();
    bool isStopped();

};

// } // namespace ROAM

#endif
