// ROS
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<sensor_msgs/PointCloud2.h>
// C++, std, boost headers
#include<iostream>

// third party headers

// 自定义头文件 
#include"Options.h"
#include"LidarOdomWrapper.h"


// global variables



int main(int argc, char** argv)
{
    ros::init(argc, argv, "LidarOdom");
    // ros::NodeHandle nh_global;
    ros::NodeHandle nh("~");

    //构造参数，把参数从launch parameters放入结构体options中
    Options options;
    readParams(nh, options);

    //实例化LidarOdomWrapper类对象，基于options注册topics回调函数；
    LidarOdomWrapper lo_wrapper(nh, options);

    ros::spin();
    return 0;
}

