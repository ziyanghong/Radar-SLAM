#include"Feature.h"

long unsigned int Feature::nNextId=0;

Feature::Feature(const cv::Mat &Pos, Frame* pFrame):mnFirstFrame(pFrame->mnId), mbBad(false), mbCorrected(false),
                                                    mnBALocalForFrame(-1),nObs(1)
                                                    // mnBALocalForFrame(pFrame->mnId)
{
    Pos.copyTo(mWorldPos);
    mnId=nNextId++;    
}

void Feature::AddObservation(long unsigned int mnIdFrame, size_t idx)
{
    if (mObservations.count(mnIdFrame))
        return;
    mObservations[mnIdFrame] = idx;
    nObs++;
}

void Feature::SetWorldPos(const cv::Mat &_WorldPos){
    _WorldPos.copyTo(mWorldPos);
}

void Feature::SetBadFlag()
{
    mbBad = true;
}

bool Feature::isBad()
{
    return mbBad;
}