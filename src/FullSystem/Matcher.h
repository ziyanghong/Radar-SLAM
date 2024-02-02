#ifndef MATCHER_H
#define MATCHER_H


#include "KDTree.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"

// namespace ROAM{

class KDTree;
class KDNode;
class Frame;
class MapPoint;
class KeyFrame;

class Matcher{
public:
    int descriptorOption; // # 1: CFAR, 2: SURF 3: ORB
public:
    Matcher(ConfigLoader config_loader);
    void SearchByProjection(Frame F, const std::vector<MapPoint*> &vpMapPoints, const float radiusMeters);

private:

};

// } // namespace ROAM

#endif