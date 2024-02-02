#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include <opencv2/core/core.hpp>

// namespace ROAM{

class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap);

    void AddObservation(KeyFrame *pKF, size_t idx);
    void EraseObservation(KeyFrame *pKF);
    bool isBad();

    // Getter
    std::map<KeyFrame*, std::size_t> GetObservations();
    cv::Mat GetWorldPos();
    cv::Mat GetDescriptor();

    // Setter
    void SetWorldPos(const cv::Mat &WorldPos);
    void SetDescriptor(const cv::Mat &descriptor);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstFrame;
    int nObs; // number of observations in common frame and keyframe

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;    
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    bool mbUpdated; // Check a point whether had been modified

    // Variables usedb by loop closing
    long unsigned int mnCorrectedByKF;
 

    static std::mutex mGlobalMutex;

protected:
    void CreateNewKeyFrame();

    // Position in absolute coordinates
    cv::Mat mWorldPos;

    // Keyframes observing the point and associated index in keyframe
    std::map<KeyFrame *, size_t> mObservations;

    // Best descriptor to fast matching
    cv::Mat mDescriptor;

    // Reference KeyFrame
    KeyFrame *mpRefKF;

    // Bad flag 
    bool mbBad;

    Map *mpMap;

    std::mutex mMutexPos;
    std::mutex mMutexFeatures;    
};

// } // namespace ROAM

#endif // MAPPOINT_H
