#include"MapPoint.h"

// namespace ROAM{

long unsigned int MapPoint::nNextId=0;
std::mutex MapPoint::mGlobalMutex;

MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap):
    mpRefKF(pRefKF), mpMap(pMap), mnLastFrameSeen(0), mnCorrectedByKF(0), mbUpdated(true), nObs(0)
{
    Pos.copyTo(mWorldPos);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    // std::unique_lock<std::mutex> lock(mpMap->mMutexPointCreation);
    mnId=nNextId++;    
}
    
// Add observation for the map point
void MapPoint::AddObservation(KeyFrame* pKF,size_t idx){
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;
    nObs++;
}

// ToDo
void MapPoint::EraseObservation(KeyFrame* pKF){

}

// ToDo: more criterias to be set
bool MapPoint::isBad(){
    mbBad = false;
    return mbBad;
}

std::map<KeyFrame*, std::size_t> MapPoint::GetObservations()
{
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mObservations;
}

cv::Mat MapPoint::GetWorldPos(){
    std::unique_lock<std::mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat MapPoint::GetDescriptor(){
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

void MapPoint::SetWorldPos(const cv::Mat &_WorldPos){
    std::unique_lock<std::mutex> lock(mMutexPos);
    _WorldPos.copyTo(mWorldPos);
    // mbUpdated = true;
}

void MapPoint::SetDescriptor(const cv::Mat &_Descriptor){
    std::unique_lock<std::mutex> lock(mMutexFeatures);
    _Descriptor.copyTo(mDescriptor);

}

// } // namespace ROAM