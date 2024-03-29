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
#include<cv_bridge/cv_bridge.h>
// C++
#include<vector>
#include<queue>
#include<map>
#include<thread>
#include<mutex>
#include<condition_variable>
#include<cmath>
// PCL
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl_conversions/pcl_conversions.h>
// 自定义工程
#include"liv_Options.h"
#include"liv_time.h"
#include"liv_utils.h"
#include"LidarOdom.h"

class LidarOdomWrapper{
  public:
    explicit LidarOdomWrapper(ros::NodeHandle& nh, Options& options);
    ~LidarOdomWrapper();

    LidarOdomWrapper(const LidarOdomWrapper&) = delete;
    LidarOdomWrapper& operator=(const LidarOdomWrapper&) = delete;


    // 作用：将数据放入队列，调用下发函数。
    // 要求：imu回调和pointcloud回调必须在同一个线程执行，不能并发。
    void addImu(const sensor_msgs::ImuConstPtr& msg);
    void addPointcloud(const sensor_msgs::PointCloud2ConstPtr& msg);

  private:

    bool tryConvertROSCloudToRingTimeCloud(
            sensor_msgs::PointCloud2ConstPtr& ros_cloud, 
            pcl::PointCloud<PointType>& out_cloud);

    inline void checkAndDispatch();
    inline void trimImuQueue();
    inline void trimPointcloudQueue();
    //
    Options options_;
    ros::NodeHandle& nh_;
    ros::Subscriber subImu;
    ros::Subscriber subPointcloud;
    // ros::Publisher features_pub;
    // ros::Publisher odom_pub;
    ros::Publisher pubRangeImg;
    ros::Publisher pubTempCloud;

    // data queue
    std::queue<sensor_msgs::ImuConstPtr> imu_queue_;
    std::queue<sensor_msgs::PointCloud2ConstPtr> pointcloud_queue_;

    // others
    LidarOdom lidar_odom_;
    FixedRatioSampler imu_warn_sampler;
    bool isInitialized = false;
    FixedRatioSampler show_cvImg_sampler;
};



#endif // LIDAR_ODOM_WRAPPER_H_