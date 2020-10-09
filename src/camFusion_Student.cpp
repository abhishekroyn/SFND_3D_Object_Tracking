
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // loop through each of the keypoint matches
    // get the keypoint index for current frame using keypoint match
    // get the keypoint for current frame using keypoint index
    // check if bounding box for current frame contains the keypoint
    // get the keypoint index for previous frame using keypoint match
    // get the keypoint for previous frame using keypoint index
    // calculate euclidean distance between keypoints from current frame and previous frame
    // store the euclidean distance in a vector array
    // repeat the same for rest of the keypoint matches
    // calculate mean value of all the stored euclidean distances
    // loop through each of the keypoint matches
    // get the keypoint index for current frame using keypoint match
    // get the keypoint for current frame using keypoint index
    // check if bounding box for current frame contains the keypoint
    // get the keypoint index for previous frame using keypoint match
    // get the keypoint for previous frame using keypoint index
    // calculate euclidean distance between keypoints from current frame and previous frame
    // calculate enhanced mean value of all the stored euclidean distances by including a multiplication factor
    // check if the euclidean distance is less than this enhanced mean value
    // store the keypoint for current frame with bounding box for current frame
    // store the keypoint matches for current frame with bounding box for current frame
    // repeat the same for rest of the keypoint matches

    std::vector<double> euclidDist;

    for(auto it = kptMatches.begin(); it != kptMatches.end(); it++)
    {
        int kptsCurrIndex = (*it).trainIdx;
        const auto &keyPointCurr = kptsCurr[kptsCurrIndex];

        if(boundingBox.roi.contains(keyPointCurr.pt))
        {
            int kptsPrevIndex = (*it).queryIdx;
            const auto &keyPointPrev = kptsPrev[kptsPrevIndex];

            euclidDist.push_back(cv::norm(keyPointCurr.pt - keyPointPrev.pt));
        }
    }

    double euclidDistMeanMultiplier = 1.5;

    int euclidDistCount =  euclidDist.size();
    double euclidDistMean = std::accumulate(euclidDist.begin(), euclidDist.end(), 0.0) / euclidDistCount;

    for(auto it = kptMatches.begin(); it != kptMatches.end(); it++)
    {
        int kptsCurrIndex = (*it).trainIdx;
        const auto &keyPointCurr = kptsCurr[kptsCurrIndex];

        if(boundingBox.roi.contains(keyPointCurr.pt))
        {
            int kptsPrevIndex = (*it).queryIdx;
            const auto &keyPointPrev = kptsPrev[kptsPrevIndex];

            double euclidDistTemp = cv::norm(keyPointCurr.pt - keyPointPrev.pt);

            double euclidDistMeanEnhanced = euclidDistMean * euclidDistMeanMultiplier;
            if(euclidDistTemp < euclidDistMeanEnhanced)
            {
                boundingBox.keypoints.push_back(keyPointCurr);
                boundingBox.kptMatches.push_back(*it);
            }
        }
    }
    std::cout << "euclidean distance - mean value : " << euclidDistMean << " keypoint matches - count before filtering : " << euclidDistCount << " count after filtering " << boundingBox.keypoints.size() << std::endl;
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // loop through each of the keypoint matches in current frame
    // get current keypoint and its matched partner in the prev. frame for outer loop
    // loop through each of the keypoint matches in inner loop
    // set minimum distance for valid keypoint distances in same frame
    // get next keypoint and its matched partner in the prev. frame for inner loop
    // compute distance between keypoints in current frame for outler loop and inner loop
    // compute distance between keypoints in previous frame for outler loop and inner loop
    // check if distances for previous frame and current frame are valid respectively
    // calculate distance ratio using distances from previous frame and current frame
    // store the distance ratio in vector array
    // repeat the same for rest of the keypoint matches in inner loop
    // repeat the same for rest of the keypoint matches in outer loop
    // check if distance ratio vector array contains any value
    // sort distance ratios
    // calculate median distance ratio
    // calculate time between two measurements in seconds
    // calculate TTC based on equation derived in the tutorial

    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // compute camera-based TTC from distance ratios
    //double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
    // replacement for meanDistRatio
    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    double dT = 1 / frameRate;
    //TTC = -dT / (1 - meanDistRatio);
    TTC = -dT / (1 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // assume width of the ego lane
    // remove lidar points outside the ego lane, if any
    // store the lidar points in heap data structure
    // sort the lidar points in order
    // choose first 100 lidar points to determine their mean location
    // do the same for both previous and current lidar points to find respective closest mean distances
    // calculate time between two measurements in seconds
    // calculate TTC based on equation derived in the tutorial

    double laneWidth = 4.0;                 // assumed width of the ego lane

    for (auto it1 = lidarPointsPrev.begin(); it1 != lidarPointsPrev.end(); ++it1)
    {
        if (abs(it1->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            lidarPointsPrev.erase(it1);
            it1--;
        }
    }

    for (auto it2 = lidarPointsCurr.begin(); it2 != lidarPointsCurr.end(); ++it2)
    {
        if (abs(it2->y) <= laneWidth / 2.0)
        { // 3D point within ego lane?
            lidarPointsCurr.erase(it2);
            it2--;
        }
    }

    unsigned int closestPointsCount = 100;

    auto compOperator = [](const LidarPoint &lp1, const LidarPoint &lp2){return lp1.x > lp2.x;};

    std::make_heap(lidarPointsPrev.begin(), lidarPointsPrev.end(), comp);
    std::sort_heap(lidarPointsPrev.begin(), lidarPointsPrev.begin() + closestPointsCount, comp);

    std::make_heap(lidarPointsCurr.begin(), lidarPointsCurr.end(), comp);
    std::sort_heap(lidarPointsCurr.begin(), lidarPointsCurr.begin() + closestPointsCount, comp);

    auto sumLidarPointX = [](const double sumLidarPoint, const LidarPoint &lp) {return sumLidarPoint + lp.x;};
    double meanXPrev = std::accumulate(lidarPointsPrev.begin(), lidarPointsPrev.begin() + closestPointsCount, 0.0, sumLidarPointX) / closestPointsCount;
    double meanXCurr = std::accumulate(lidarPointsCurr.begin(), lidarPointsCurr.begin() + closestPointsCount, 0.0, sumLidarPointX) / closestPointsCount;

    double dT = 1.0 / frameRate;
    TTC = meanXCurr * dT / (meanXPrev - meanXCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // loop through each of the bounding boxes in previous frame
    // loop through each of the matches
    // filter out matches belonging to a particular bounding box in previous frame
    // loop throught each of the filtered matches
    // loop through each of the bounding boxes in current frame
    // associate each of the bounding boxes in current frame to their corresponding matched keypoints
    // find out bounding box in current frame having maximum occurences of the matched keypoints
    // associate bounding box from previous frame to the bounding box in current frame
    // repeat the process for each of the bounding boxes in previous frame to associate it with the corresponding bounding box in current frame

    for(auto it = prevFrame.boundingBoxes.begin(); it != prevFrame.boundingBoxes.end(); it++)
    {
        std::vector<vector<cv::DMatch>::iterator> enclosedMatches;
        for(auto it1 = matches.begin(); it1 != matches.end(); it1++)
        {
            int prevFrameKeyPointIdx = it1->queryIdx;
            if(it->roi.contains(prevFrame.keypoints.at(prevFrameKeyPointIdx).pt))
            {
                enclosedMatches.push_back(it1);
            }
        }

        // contains a sorted list of key-value pairs, while permitting multiple entries with the same key
        std::multimap<int, int> boxIdPairKeyPointIdx;

        for(auto it2 = enclosedMatches.begin(); it2 != enclosedMatches.end(); it2++)
        {
            int currFrameKeyPointIdx = (*it2)->trainIdx;
            for(auto it3 = currFrame.boundingBoxes.begin(); it3 != currFrame.boundingBoxes.end(); it3++)
            {
                if(it3->roi.contains(currFrame.keypoints.at(currFrameKeyPointIdx).pt))
                {
                    int currFrameBoxId = it3->boxID;
                    boxIdPairKeyPointIdx.insert(std::pair<int, int>(currFrameBoxId, currFrameKeyPointIdx));
                }
            }
        }

        int currFrameKeyPointMaxCount = 0;
        int currFrameBoxIdBestIndex = 1e8;

        if(boxIdPairKeyPointIdx.size() > 0)
        {
            for(auto it4 = boxIdPairKeyPointIdx.begin(); it4 != boxIdPairKeyPointIdx.end(); it4++)
            {
                if(boxIdPairKeyPointIdx.count(it4->first) > currFrameKeyPointMaxCount)
                {
                    currFrameKeyPointMaxCount = boxIdPairKeyPointIdx.count(it4->first);
                    currFrameBoxIdBestIndex = it4->first;
                }
            }
            bbBestMatches.insert(std::pair<int, int>(it->boxID, currFrameBoxIdBestIndex));
        }
    }
}
