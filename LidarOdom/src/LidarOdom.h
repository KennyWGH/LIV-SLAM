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
// 自定义工程
#include"Options.h"
#include"liv_time.h"
#include"liv_utils.h"
#include"ImuTracker.h"

using namespace std;

class LidarOdom{
  public:
    explicit LidarOdom(LidarOdomOptions& lo_options);
    ~LidarOdom();

    LidarOdom(const LidarOdom&) = delete;
    LidarOdom& operator=(const LidarOdom&) = delete;

    void addImu(ImuData& source_imu);
    void addPointcloud(pcl::PointCloud<pcl::PointXYZ>& source_cloud);

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
    // containers
    std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> map_queue_;
};

#endif // LIDAR_ODOM_H_