/**
 * 所有数据成员应尽可能地设置默认参数；
 * 
 */

#ifndef OPTIONS_H_
#define OPTIONS_H_

// ROS
#include<ros/ros.h>
// c++, thirdparty, etc.
#include<iostream>
#include<Eigen/Core>

struct FeatureExtractorOptions
{
    //
};

struct LidarOdomOptions
{
    /* LiDAR data params */
    double max_range = 60.0;
    double min_range = 0.4;
    /* ImuTracker */
    double max_imu_interval = 0.5;              /* 允许的最大imu数据间隔，超过则报警 */
    double imu_gravity_constant = 9.806;        /* 重力常量 */
    double imu_gravity_time_constant = 10.;     /* 一节平滑滤波参数 */
    /* ImuTracker: Cubic Spline Interpolation */
    //
    /* Dynamic obj filter */
    bool use_DynObjFilter = false;
    //
    /* motion distortion */
    bool use_pointcloud_deskew = false;
};

struct Options
{
    /*  */
    std::string imu_topic = "/imu_data";
    std::string pointcloud_topic = "/velodyne_points";
    /* data queue maximum duration */
    double imu_queue_duration = 5.;          /* imu data 队列最大时长 */
    double pointcloud_queue_duration = 5.;   /* pointcloud data 队列最大时长 */
    /* imu TF relative to LiDAR */
    double x_offset = 0.;
    double y_offset = 0.;
    double z_offset = 0.;
    double roll_offset = 0.;
    double pitch_offset = 0.;
    double yaw_offset = 0.;

    LidarOdomOptions lidar_odom_options;
};


void readParams(ros::NodeHandle& nh_, Options& options_);






#endif // OPTIONS_H_