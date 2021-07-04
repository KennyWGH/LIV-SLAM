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
#include<queue>
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

    void addImu(const ImuData& source_imu);
    void addPointcloud(const pcl::PointCloud<pcl::PointXYZ>& source_cloud);

    const cv::Mat getCurRangeImg(){return cur_range_img;}

  private:

    void ndtMatch(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
            const Eigen::Matrix4f& init_guess,
            Eigen::Matrix4f& pose_estimate);
    void featureMatch();

    enum WhichMethod {
        NDT = 0,
        FEATURE = 1,
        OTHER = 2
    };

    LidarOdomOptions lo_options_;
    WhichMethod which_method = NDT;
    ImuTracker imu_tracker_;
    FeatureExtractor f_extractor_;
    // containers
    std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> map_queue_;
    // run time variables
    cv::Mat cur_range_img;
};

#endif // LIDAR_ODOM_H_