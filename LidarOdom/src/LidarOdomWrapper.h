/**
 * 
 * 
 */

#ifndef LIDAR_ODOM_WRAPPER_H_
#define LIDAR_ODOM_WRAPPER_H_

// ROS
#include<ros/ros.h>
#include<std_msgs/Bool.h>
#include<std_msgs/String.h>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
// C++
#include<vector>
#include<queue>
#include<map>
#include<thread>
#include<mutex>
#include<condition_variable>
#include<cmath>
// 自定义工程
#include"Options.h"
#include"time.h"
#include"liv_utils.h"
#include"LidarOdom.h"

class LidarOdomWrapper{
  public:
    explicit LidarOdomWrapper(ros::NodeHandle& nh, Options& options);
    ~LidarOdomWrapper();

    LidarOdomWrapper(const LidarOdomWrapper&) = delete;
    LidarOdomWrapper& operator=(const LidarOdomWrapper&) = delete;

    void addImu(const sensor_msgs::ImuConstPtr& msg);
    void addPointcloud(const sensor_msgs::PointCloud2ConstPtr& msg);

  private:
    //
    // inline Time_N fromRosTime(ros::Time ros_time)
    // {
    //     return Time_N(3000);
    // }

    // inline ros::Time toRosTime(Time_N time){
    //     //
    // }

    // void fromRosPointCloud2(sensor_msgs::PointCloud2ConstPtr pointcloud2);
    // sensor_msgs::PointCloud2ConstPtr toRosPointCloud2();

    inline void checkAndDispatch();
    inline void trimImuQueue();
    inline void trimPointcloudQueue();
    //
    Options options_;
    ros::NodeHandle& nh_;
    ros::Subscriber imu_sub;
    ros::Subscriber pointcloud_sub;
    // ros::Publisher features_pub;
    // ros::Publisher odom_pub;

    // data queue
    std::queue<sensor_msgs::ImuConstPtr> imu_queue_;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointcloud_queue_;

    // others
    FixedRatioSampler imu_warn_sampler;
    bool isInitialized = false;
};



#endif // LIDAR_ODOM_WRAPPER_H_