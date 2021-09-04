/**
 * 
 * 
 * 
 * 
 */

#ifndef LIDAR_ODOM_H_
#define LIDAR_ODOM_H_

// C++, third party
#include<string>
#include<vector>
#include<deque>
// PCL
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
// OpenCV
#include<opencv/cv.hpp>
// 自定义工程
#include"liv_Options.h"
#include"liv_time.h"
#include"liv_utils.h"
#include"ImuTracker.h"
#include"FeatureExtractor.h"

using namespace std;

class LidarOdom{
  public:
    explicit LidarOdom(LidarOdomOptions& lo_options);
    ~LidarOdom();

    LidarOdom(const LidarOdom&) = delete;
    LidarOdom& operator=(const LidarOdom&) = delete;

    /* query interfaces */
    const cv::Mat getCurRangeImg(){return currRangeImg;}
    // const void getKeyPoses();

    /* query interfaces */
    void addImu(const ImuData& source_imu);
    void addPointcloud(const common::Time& time_stamp, 
            const pcl::PointCloud<PointType>& source_cloud);


  private:

    void CalculateCloudInfo(pcl::PointCloud<PointType>::Ptr& input_cloud_);

    void ndtMatch(pcl::PointCloud<PointType>::Ptr target_cloud,
            pcl::PointCloud<PointType>::Ptr input_cloud,
            const Eigen::Matrix4f& init_guess,
            Eigen::Matrix4f& pose_estimate);
    void featureMatch();

    void updateRecentPoses(Rigid3d& latest_pose);
    Eigen::Vector3d getRecentTranslation();

    enum WhichMethod {
        NDT = 0,
        FEATURE = 1,
        OTHER = 2
    };

    // significant members
    LidarOdomOptions lo_options_;
    ImuTracker imu_tracker_;
    // FeatureMatcher f_matcher_;
    FeatureExtractor f_extractor_;

    // significant containers
    std::deque<pcl::PointCloud<PointType>::Ptr> filteredCloudKeyFrames;
    std::deque<CompletePose> filteredCloudKeyPoses;
    pcl::PointCloud<PointType>::Ptr globalMapDS;

    // run-time state variables
    cv::Mat currRangeImg;
    std::size_t num_received_cloud = 0;
    std::size_t num_keyframe_cloud = 0;

    pcl::PointCloud<PointType>::Ptr newCloudOriginal;
    CompletePose newPose;

    pcl::PointCloud<PointType>::Ptr currCloudOriginal;
    pcl::PointCloud<PointType>::Ptr currCloudFiltered;
    pcl::PointCloud<PointType>::Ptr currCloudDynamicd;
    CompletePose currPose;

    // common::Time lastCloudTime;
    // Rigid3d lastPo0se;
    pcl::PointCloud<PointType>::Ptr lastCloudOriginal;
    pcl::PointCloud<PointType>::Ptr lastCloudFiltered;
    CompletePose lastPose;

    Rigid3d initial_guess;

    Rigid3d currMinus1Pose;
    Rigid3d currMinus2Pose;
};

#endif // LIDAR_ODOM_H_