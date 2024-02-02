#ifndef FEATURE_H
#define FEATURE_H

#include "MapPoint.h"
#include "Frame.h"
#include "../util/Loader.h"

class Frame;

class Feature
{
public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstFrame;
    int nObs; // number of observations in the window

    // Variables used by local mapping

    // check if the feature has been added in the list for local BA feature list. 
    // If added already,  it should be equal to the current frame ID in the tracker.
    // It is initiliazed with the first observed frame ID.
    long unsigned int mnBALocalForFrame; 

    // Variables used by loop closing
    bool mbCorrected;

    // Position in absolute coordinates
    cv::Mat mWorldPos; // WorldPos.at<float>(0, 0), WorldPos.at<float>(1, 0)

    // frame observing the point and associated index in frame
    std::map<long unsigned int, size_t> mObservations;   

    Feature();
    Feature(const cv::Mat &Pos, Frame* pFrame);

    void SetWorldPos(const cv::Mat &_WorldPos);
    void AddObservation(long unsigned int frameIdx,size_t idx);

    void SetBadFlag();
    bool isBad();


private:
    bool mbBad;
};

#endif