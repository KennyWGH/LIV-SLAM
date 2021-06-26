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
// PCL
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
// 自定义工程
#include"Options.h"
#include"liv_time.h"
#include"liv_utils.h"

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
    enum WhichMethod {
        NDT = 0,
        FEATURE = 1,
        OTHER = 2
    };

    LidarOdomOptions lo_options_;
    WhichMethod which_method = NDT;
};

#endif // LIDAR_ODOM_H_