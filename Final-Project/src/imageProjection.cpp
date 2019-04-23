#include "utility.h"
#include <cmath>

class ImageProjection{
private:
    // Node
    ros::NodeHandle n;
    // Sensor message
    ros::Subscriber subLaserPcd;
    ros::Publisher pubFullPcd;
    ros::Publisher pubPcdWithDist;
    ros::Publisher pubGroundPcd;
    ros::Publisher pubSegmentedPcd;
    ros::Publisher pubSegmentedPcdPure;
    ros::Publisher pubSegmentedPcdInfo;
    ros::Publisher pubOutlierCloud;
    // Point Cloud
    pcl::PointCloud<PointType>::Ptr laserPcd;
    pcl::PointCloud<PointType>::Ptr fullPcd; // projected point cloud 
    pcl::PointCloud<PointType>::Ptr PcdWithDist;
    pcl::PointCloud<PointType>::Ptr groundPcd; 
    pcl::PointCloud<PointType>::Ptr segmentedPcd;
    pcl::PointCloud<PointType>::Ptr segmentedPcdPure;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanpt;

    cv::Mat distMat;
    cv::Mat labelMat;
    cv::Mat groundMat;
    int labelCount;

    float startOrientation;
    float endOrientation;

    cloud_msgs::cloud_info PcdMsg;
    std_msgs::Header cloudHeader;

    std::vector<std::pair<uint8_t, uint8_t> > neighborIterator;

    uint16_t *allPushedIndX;
    uint16_t *allPushedIndY;

    uint16_t *queueIndX;
    uint16_t *queueIndY;

public:
    ImageProjection():
        n("~"){

        subLaserPcd = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &ImageProjection::cloudHandler, this);

        pubFullPcd = n.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", 1);
        pubPcdWithDist = n.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", 1);

        pubGroundPcd = n.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", 1);
        pubSegmentedPcd = n.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", 1);
        pubSegmentedPcdPure = n.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", 1);
        pubSegmentedPcdInfo = n.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", 1);
        pubOutlierCloud = n.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", 1);
        // initialize NaN
        nanpt.x = std::numeric_limits<float>::quiet_NaN();
        nanpt.y = std::numeric_limits<float>::quiet_NaN();
        nanpt.z = std::numeric_limits<float>::quiet_NaN();
        nanpt.intensity = -1;

        reset_pointcloud();
        clear_variable();
    }

    void reset_pointcloud(){
        // reset those to new point cloud
        laserPcd.reset(new pcl::PointCloud<PointType>());
        fullPcd.reset(new pcl::PointCloud<PointType>());
        PcdWithDist.reset(new pcl::PointCloud<PointType>());
        groundPcd.reset(new pcl::PointCloud<PointType>());
        segmentedPcd.reset(new pcl::PointCloud<PointType>());
        segmentedPcdPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());

        fullPcd->points.resize(N_SCAN*Horizon_SCAN);
        PcdWithDist->points.resize(N_SCAN*Horizon_SCAN);
        // assign PcdMsg
        PcdMsg.startRingIndex.assign(N_SCAN, 0);
        PcdMsg.endRingIndex.assign(N_SCAN, 0);
        PcdMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        PcdMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        PcdMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);
        // reset neighbor
        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        // reset Index to new uint16_t
        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    void clear_variable(){
        // clear point cloud
        laserPcd->clear();
        groundPcd->clear();
        segmentedPcd->clear();
        segmentedPcdPure->clear();
        outlierCloud->clear();
        // reset Mat
        distMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;
        // fill all fullPcd & PcdWithDist with NaN
        std::fill(fullPcd->points.begin(), fullPcd->points.end(), nanpt);
        std::fill(PcdWithDist->points.begin(), PcdWithDist->points.end(), nanpt);
    }

    ~ImageProjection(){}

    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;
        pcl::fromROSMsg(*laserCloudMsg, *laserPcd);
    }
    
    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        copyPointCloud(laserCloudMsg);
        findStartEndAngle();
        projectPointCloud();
        groundRemoval();
        cloudSegmentation();
        publishCloud();
        clear_variable();
    }

    void findStartEndAngle(){
        PcdMsg.startOrientation = -atan2(laserPcd->points[0].y, laserPcd->points[0].x);
        PcdMsg.endOrientation   = -atan2(laserPcd->points[laserPcd->points.size() - 1].y,
                                                     laserPcd->points[laserPcd->points.size() - 2].x) + 2 * M_PI;
        if (PcdMsg.endOrientation - PcdMsg.startOrientation > 3 * M_PI) {
            PcdMsg.endOrientation -= 2 * M_PI;
        } 
        else if (PcdMsg.endOrientation - PcdMsg.startOrientation < M_PI){
            PcdMsg.endOrientation += 2 * M_PI;
        }
        PcdMsg.orientationDiff = PcdMsg.endOrientation - PcdMsg.startOrientation;
    }

    void groundRemoval(){
        // degree difference in 10 degree -> groundCloud
        size_t lowerIdx, upperIdx;
        float dx, dy, dz, angle;

        for (size_t j = 0; j < Horizon_SCAN; ++j){
            for (size_t i = 0; i < groundScanInd; ++i){

                lowerIdx = i * Horizon_SCAN + j;
                upperIdx = (i+1) * Horizon_SCAN + j;

                if (fullPcd->points[lowerIdx].intensity == -1 ||
                    fullPcd->points[upperIdx].intensity == -1){
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                    
                dx = fullPcd->points[upperIdx].x - fullPcd->points[lowerIdx].x;
                dy = fullPcd->points[upperIdx].y - fullPcd->points[lowerIdx].y;
                dz = fullPcd->points[upperIdx].z - fullPcd->points[lowerIdx].z;
                // vertical angle in degree
                angle = atan2(dz, sqrt(pow(dx, 2) + pow(dy, 2))) * 180 / M_PI;

                if (abs(angle - sensorMountAngle) <= 10){
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }

        for (size_t i = 0; i < N_SCAN; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1 || distMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;
                }
            }
        }
        if (pubGroundPcd.getNumSubscribers() != 0){
            for (size_t i = 0; i <= groundScanInd; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (groundMat.at<int8_t>(i,j) == 1)
                        groundPcd->push_back(fullPcd->points[j + i*Horizon_SCAN]);
                }
            }
        }
    }

    void cloudSegmentation(){
        // takes a single scan’s point cloud and projects it onto a range image for segmentation.
        for (size_t i = 0; i < N_SCAN; ++i)
            for (size_t j = 0; j < Horizon_SCAN; ++j)
                if (labelMat.at<int>(i,j) == 0)
                    labelComponents(i, j);

        int sizeOfSegCloud = 0;
        for (size_t i = 0; i < N_SCAN; ++i) {

            PcdMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;

            for (size_t j = 0; j < Horizon_SCAN; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){
                    if (labelMat.at<int>(i,j) == 999999){
                        if (i > groundScanInd && j % 5 == 0){
                            outlierCloud->push_back(fullPcd->points[j + i*Horizon_SCAN]);
                            continue;
                        }else{
                            continue;
                        }
                    }
                    if (groundMat.at<int8_t>(i,j) == 1){
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)
                            continue;
                    }
                    PcdMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                    PcdMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                    PcdMsg.segmentedCloudRange[sizeOfSegCloud]  = distMat.at<float>(i,j);
                    segmentedPcd->push_back(fullPcd->points[j + i*Horizon_SCAN]);
                    ++sizeOfSegCloud;
                }
            }

            PcdMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
        }

        if (pubSegmentedPcdPure.getNumSubscribers() != 0){
            for (size_t i = 0; i < N_SCAN; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){
                        segmentedPcdPure->push_back(fullPcd->points[j + i*Horizon_SCAN]);
                        segmentedPcdPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }

    void projectPointCloud(){
        // Project point cloud
        float verticalAngle, horizonAngle, dist;
        size_t rowIdx, columnIdx, index, cloudSize; 
        PointType currentPt;

        cloudSize = laserPcd->points.size();

        for (size_t i = 0; i < cloudSize; ++i){
            // obtain current point coordinates
            currentPt.x = laserPcd->points[i].x;
            currentPt.y = laserPcd->points[i].y;
            currentPt.z = laserPcd->points[i].z;
            // vertical angle in degree
            verticalAngle = atan2(currentPt.z, sqrt(pow(currentPt.x, 2) + pow(currentPt.y, 2))) * 180 / M_PI;
            // horizontal angle in degree
            horizonAngle = atan2(currentPt.x, currentPt.y) * 180 / M_PI;
            // convert verticalAngle & horizonAngle to rowIdx & columnIdx
            rowIdx = (verticalAngle + ang_bottom) / ang_res_y;
            columnIdx = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdx >= Horizon_SCAN)
                columnIdx -= Horizon_SCAN;
            index = rowIdx * Horizon_SCAN + columnIdx;

            // skip this point if index unreasonable
            if (rowIdx < 0 || rowIdx >= N_SCAN)
                continue;
            if (columnIdx < 0 || columnIdx >= Horizon_SCAN)
                continue;
            
            // calculate distance and save in distMat & PcdWithDist
            dist = sqrt(pow(currentPt.x, 2) + pow(currentPt.y, 2) + pow(currentPt.z, 2));
            distMat.at<float>(rowIdx, columnIdx) = dist;
            PcdWithDist->points[index].intensity = dist;
            // save into Pcd->points[index]
            currentPt.intensity = (float)rowIdx + (float)columnIdx / 10000.0;
            fullPcd->points[index] = currentPt;
            
        }
    }

    void labelComponents(int row, int col){
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};

        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        
        while(queueSize > 0){
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            labelMat.at<int>(fromIndX, fromIndY) = labelCount;

            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){

                thisIndX = fromIndX + (*iter).first;
                thisIndY = fromIndY + (*iter).second;

                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;

                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;

                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;

                d1 = std::max(distMat.at<float>(fromIndX, fromIndY), 
                              distMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(distMat.at<float>(fromIndX, fromIndY), 
                              distMat.at<float>(thisIndX, thisIndY));

                if ((*iter).first == 0)
                    alpha = segmentAlphaX;
                else
                    alpha = segmentAlphaY;

                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));

                if (angle > segmentTheta){

                    queueIndX[queueEndInd] = thisIndX;
                    queueIndY[queueEndInd] = thisIndY;
                    ++queueSize;
                    ++queueEndInd;

                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                }
            }
        }


        bool feasibleSegment = false;
        if (allPushedIndSize >= 30)
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum){
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)
                if (lineCountFlag[i] == true)
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)
                feasibleSegment = true;            
        }

        if (feasibleSegment == true){
            ++labelCount;
        }else{
            for (size_t i = 0; i < allPushedIndSize; ++i){
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    
    void publishCloud(){

        PcdMsg.header = cloudHeader;
        pubSegmentedPcdInfo.publish(PcdMsg);

        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);

        pcl::toROSMsg(*segmentedPcd, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedPcd.publish(laserCloudTemp);

        if (pubFullPcd.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullPcd, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullPcd.publish(laserCloudTemp);
        }

        if (pubGroundPcd.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundPcd, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubGroundPcd.publish(laserCloudTemp);
        }

        if (pubSegmentedPcdPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedPcdPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedPcdPure.publish(laserCloudTemp);
        }

        if (pubPcdWithDist.getNumSubscribers() != 0){
            pcl::toROSMsg(*PcdWithDist, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubPcdWithDist.publish(laserCloudTemp);
        }
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "lego_loam");
    
    ImageProjection IP;

    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");

    ros::spin();
    return 0;
}
