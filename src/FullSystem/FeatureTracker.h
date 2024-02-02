#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"
#include "Algorithms.h"
#include "../util/Loader.h"
#include "Feature.h"

class Feature;
using Grid = std::vector<float>;

class FeatureTracker
{
public:
    FeatureTracker(const ConfigLoader &config_loader);
    ~FeatureTracker();
    bool Initialized(Frame *pFrame, const ConfigLoader &config_loader);
    void TrackNewFrame(Frame *pNewFrame, Frame *pLastFrame, std::vector<Frame *> vpWindowFrames, 
                                    std::vector<cv::Mat> vRT, cv::Mat &RT,
                                    const ConfigLoader &config_loader);
    std::set<unsigned int> AddBestCandidates(std::vector<cv::Point2f> trackedPoints, std::vector<cv::Point2f> candidates,
                                             int maxAddedFeature);
    void DrawFeatures(cv::Mat image,
                      std::vector<Feature *> trackedFeatures,
                      std::vector<cv::Point2f> trackedFeatureCoordinates,
                      std::vector<Feature *> newFeatures,
                      std::vector<cv::Point2f> newFeatureCoordinates,
                      cv::Mat &visImage);
    bool isInsideGrid(Grid grid, cv::Point2f point);
    void SortGrids();

    std::set<Feature *> GetAllFeatures();
    std::set<Feature *> GetNewFeatures();
    std::vector<Feature *> GetTrackedFeatures();


public:    

    int mnTrackedPoints;

private:
    std::set<Feature *> mspFeatures;
    std::set<Feature *> mspNewFeatures;
    std::vector<Feature *> mvpTrackedFeatures;
    std::vector<Grid> mvGrids;
    std::vector<int> mvNumPointsInGrids;
};

#endif
