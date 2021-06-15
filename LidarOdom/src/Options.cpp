/**
 * 
 * 
 */

#include"Options.h"



template <typename T>
bool readParam(ros::NodeHandle& nh_, std::string name, T& value)
{
    if(nh_.getParam(name, value))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << value);
        return true;
    }else{
        ROS_WARN_STREAM("Failed to load " << name 
                    << "   [set default value: " << value << " ]");
        return false;
    }
}


void readParams(ros::NodeHandle& nh_, Options& options_)
{
    readParam<std::string>(nh_, "imu_topic", options_.imu_topic);
    readParam<std::string>(nh_, "pointcloud_topic", options_.pointcloud_topic);
    readParam<double>(nh_, "imu_queue_duration", options_.imu_queue_duration);
    readParam<double>(nh_, "pointcloud_queue_duration", options_.pointcloud_queue_duration);
    readParam<double>(nh_, "x_offset", options_.x_offset);
    readParam<double>(nh_, "y_offset", options_.y_offset);
    readParam<double>(nh_, "z_offset", options_.z_offset);
    readParam<double>(nh_, "roll_offset", options_.roll_offset);
    readParam<double>(nh_, "pitch_offset", options_.pitch_offset);
    readParam<double>(nh_, "yaw_offset", options_.yaw_offset);
    readParam<double>(nh_, "max_imu_interval", 
        options_.lidar_odom_options.max_imu_interval);
    readParam<double>(nh_, "imu_gravity_constant", 
        options_.lidar_odom_options.imu_gravity_constant);
    readParam<double>(nh_, "imu_gravity_time_constant", 
        options_.lidar_odom_options.imu_gravity_time_constant);

    ROS_INFO_STREAM("// ********** Load parameters done. ********** //");
    
    return;
}
