#include "LocalMapping.h"

// namespace ROAM{

LocalMapping::LocalMapping(){

}

LocalMapping::~LocalMapping(){

}

LocalMapping::LocalMapping(Map* pMap, ConfigLoader _config_loader): mpMap(pMap), config_loader(_config_loader)
{

}


void LocalMapping::SetTracker(Tracking* pTracker){
    mpTracker = pTracker;
}

void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void LocalMapping::Run(){

    // mbFinished = false;
    // while(1)
    // {

        // std::cout << "LocalMapping running......\n";        
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Process new keyframe and insertion in Map
            ProcessNewKeyFrame();

            // Check recent MapPoints
            MapPointCulling();

            // Create new MapPoints
            CreateNewMapPoints();

            mbAbortBA = false;

            // // Local BA
            // if(mpMap->KeyFramesInMap()>2)
            //     Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap, config_loader);

            mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

        }

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        // usleep(500);   

    // }
    // SetFinish();

}

bool LocalMapping::CheckNewKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}

bool LocalMapping::AcceptKeyFrames()
{
    std::unique_lock<std::mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void LocalMapping::SetAcceptKeyFrames(bool flag)
{
    std::unique_lock<std::mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

void LocalMapping::InsertKeyFrame(KeyFrame* pKF)
{
    std::unique_lock<std::mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);    
    mbAbortBA=true;
}

// 
void LocalMapping::ProcessNewKeyFrame()
{
    {
        std::unique_lock<std::mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }


    // Compute global descriptor, using M2DP
    // if (config_loader.loopClosure)
        // std::cout << mpCurrentKeyFrame->mvPointCloud.rows << std::endl;
        // mpCurrentKeyFrame->ComputeGlobalDescriptor(); 



    // Associate MapPoints to the new keyframe and update normal and descriptor
    const std::vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        MapPoint* pMP = vpMapPointMatches[i];
        if(pMP)
        {
            if(!pMP->isBad())
            {
                
                pMP->AddObservation(mpCurrentKeyFrame, i);
            }    
            else // this can only happen for new stereo points inserted by the Tracking
            {
                mlpRecentAddedMapPoints.push_back(pMP);
            }
        }
    }   

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->AddKeyFrame(mpCurrentKeyFrame);    
    // std::cout << "Inserted new KeyFrame into Map." <<  std::endl;

}

// std::list<KeyFrame*> LocalMapping::GetNewKeyFrames()
// {
//     return mlNewKeyFrames;
// }

/**
 * Create new map points
 * from those appeared in new keyframe but not matched with previous keyframe
 */
void LocalMapping::CreateNewMapPoints()
{
    mlpRecentAddedMapPoints.clear();

    std::vector<cv::KeyPoint> vpKPs = mpCurrentKeyFrame->mvKeypoints;
    std::vector<MapPoint*> vpMPs = mpCurrentKeyFrame->GetMapPointMatches();   
    cv::Mat vDescriptors = mpCurrentKeyFrame->mvDescriptors;
    // std::cout << "vDescriptors rows: " << vDescriptors.rows << " cols: " vDescriptor.cols << std::endl; 

    int index = 0;
    int counter = 0;
    for(std::vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
    {   
        cv::KeyPoint keypt = vpKPs[index];
        MapPoint* pMP = *vit;

        // If the keypoint is not matched with existed map point, create a new map point.
        if(!pMP)
        {   
            cv::Point2d ptLocal = imageFrame2LocalFrame(config_loader.cartImageHeight, config_loader.cartImageWidth,
                                                    config_loader.cartImageScale, keypt.pt);
            cv::Mat LocalPos = cv::Mat(3, 1, CV_32FC1);
            LocalPos.at<float>(0,0) = ptLocal.x;
            LocalPos.at<float>(1,0) = ptLocal.y;
            LocalPos.at<float>(2,0) = 1.0;   
            cv::Mat WorldPos = mpCurrentKeyFrame->GetPose() * LocalPos;         

            MapPoint* pNewMP = new MapPoint(WorldPos, mpCurrentKeyFrame, mpMap);
            pNewMP->SetDescriptor(vDescriptors.row(index));
            pNewMP->AddObservation(mpCurrentKeyFrame, index);
            mpCurrentKeyFrame->AddMapPoint(pNewMP, index);
            mpMap->AddMapPoint(pNewMP);
            mlpRecentAddedMapPoints.push_back(pNewMP);
            counter++;
        }

        index++;
    }
    // std::cout << "LocalMapping::Number of keypoints in this keyframe: " << vpKPs.size() << std::endl;
    // std::cout << "LocalMapping::Created " << counter << " new MapPoint." << std::endl;
}


// Delete recent added map point if it is only seen once by the keyframe which owns it
void LocalMapping::MapPointCulling()
{   
    
    for(std::list<MapPoint*>::iterator vit=mlpRecentAddedMapPoints.begin(), vend=mlpRecentAddedMapPoints.end(); vit!=vend; vit++){
        MapPoint* pMP = *vit;
        if (pMP->nObs < 2){
            mpMap->DeleteMapPoint(pMP);
        }
    }
}

// Todo
bool LocalMapping::CheckFinish()
{
    return true;
}

// Todo
void LocalMapping::SetFinish()
{

}

void LocalMapping::RequestStop()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    mbStopRequested = true;
    std::unique_lock<std::mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool LocalMapping::isStopped()
{
    std::unique_lock<std::mutex> lock(mMutexStop);
    return mbStopped;
}

// } // namespace ROAM
