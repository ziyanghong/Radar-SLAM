#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "Frame.h"
#include "KeyFrame.h"
#include <set>

#include <mutex>

// namespace ROAM{

class MapPoint;
class KeyFrame;

class Map
{
public:
    Map();

    void AddOriginKeyFrame(KeyFrame* pKF);
    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);

    void DeleteMapPoint(MapPoint* pMP);
    std::vector<MapPoint*> mvDeleteMapPointQueue;

    long unsigned int MapPointsInMap();
    long unsigned  KeyFramesInMap();

    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<KeyFrame*> GetAllKeyFrames();

    void SetReferenceMapPoints(std::vector<MapPoint*> vpLocalMapPoints);
    std::vector<MapPoint*> mvpLocalMapPoints;

    // For Viewer: include new map point and the map point after optimization
    std::vector<MapPoint*> mvpUpdatedMapPoints;

    KeyFrame* OriginKeyFrame;
    
    std::mutex mMutexMapUpdate;

protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::mutex mMutexMap;
};

// } // namespace ROAM

#endif // MAP_H