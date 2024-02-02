#include"Matcher.h"

// namespace ROAM{

Matcher::Matcher(ConfigLoader config_loader){
    descriptorOption = config_loader.descriptorOption;
}


void Matcher::SearchByProjection(Frame F, const std::vector<MapPoint*> &vpMapPoints, const float radiusMeters)
{
    printf("Enter SearchByProjection\n");
    for(size_t iMP=0; iMP<vpMapPoints.size(); iMP++)
    {
        MapPoint* pMP = vpMapPoints[iMP];

        if(!pMP->mbTrackInView)
            continue;

        if(pMP->isBad())
            continue;
        
        // 2D in absolute coordinates
        cv::Mat MapPointWorldPos = pMP->GetWorldPos();
        cv::Mat P = MapPointWorldPos.rowRange(0,2);

        // 2D in local frame coordinate  
        cv::Mat Tcw = F.TcwSVD;
        cv::Mat Rcw = Tcw.rowRange(0,2).colRange(0,2);
        cv::Mat Rwc = Rcw.t();
        cv::Mat tcw = Tcw.rowRange(0,2).col(2);
        const cv::Mat Pc = Rwc*P-tcw;
        cv::Point2d local_point(Pc.at<float>(0), Pc.at<float>(1));      

        // Use KD tree to retrieve neighborhood keypoints in local frame
        point_t pt;
        pt = {local_point.x , local_point.y};
        indexArr neighborhood_indices = F.mTree.neighborhood_indices(pt, radiusMeters);

        // Find the best match
        double minDist = 10000.0;
        size_t minIndex = -1;
        cv::Mat descriptors1;
        cv::Mat descriptors2;
        descriptors1 = pMP->GetDescriptor();
        // descriptors1.convertTo(descriptors1, CV_32F);
        // std::cout << "descriptors1 rows: " << descriptors1.rows << " descriptors1 cols: " << descriptors1.cols << std::endl;

        for (int i=0; i<neighborhood_indices.size(); i++)
        {

            double dist;
            size_t index = neighborhood_indices[i];
            descriptors2 = F.mvDescriptors.row(index);
            // descriptors2.convertTo(descriptors2, CV_32F);
            // std::cout << "descriptors2 rows: " << descriptors2.rows << " descriptors2 cols: " << descriptors2.cols << std::endl;

            if (descriptorOption==2){
                dist = cv::norm(descriptors1,descriptors2,cv::NORM_L2);      // l2 for surf,sift
            }else{
                dist = cv::norm(descriptors1,descriptors2,cv::NORM_HAMMING); // for ORB,BRIEF,etc.
            }

            if (dist < minDist && F.mvpMapPoints[index]==NULL){
                minDist = dist;
                minIndex = index;
            }
        }

        // A match is found
        if (minIndex != -1){
            F.mvpMapPoints[minIndex] = pMP;
            pMP->mnLastFrameSeen = F.mnId;
            pMP->mnTrackReferenceForFrame= F.mnId;
            pMP->nObs++;
        }
    }
}

// } // namespace ROAM
