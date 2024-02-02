
#include "Map.h"


// namespace ROAM{


Map::Map(){
    
}

void Map::AddOriginKeyFrame(KeyFrame* pKF){
    OriginKeyFrame = pKF;
}

void Map::AddKeyFrame(KeyFrame* pKF){
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
}

void Map::AddMapPoint(MapPoint* pMP){
    std::unique_lock<std::mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
    mvpUpdatedMapPoints.push_back(pMP);
}

void Map::DeleteMapPoint(MapPoint* pMP){

    std::unique_lock<std::mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);
    mvDeleteMapPointQueue.push_back(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint    
}

long unsigned int Map::MapPointsInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

void Map::SetReferenceMapPoints(std::vector<MapPoint*> vpLocalMapPoints)
{
    std::unique_lock<std::mutex> lock(mMutexMap);

    mvpLocalMapPoints.clear();
    mvpLocalMapPoints = vpLocalMapPoints;
}

std::vector<KeyFrame*> Map::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

std::vector<MapPoint*> Map::GetAllMapPoints(){
    std::unique_lock<std::mutex> lock(mMutexMap);
    return std::vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

// } // namespace ROAM
