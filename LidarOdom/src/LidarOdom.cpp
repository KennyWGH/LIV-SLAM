/**
 * 描述
 * 
 * 
 * 
 */


// C++
#include<iostream>
// 自定义
#include"LidarOdom.h"
#include"liv_time.h"


LidarOdom::LidarOdom(LidarOdomOptions& lo_options)
    :lo_options_(lo_options)
{
    //
}

LidarOdom::~LidarOdom()
{
    //
}

void LidarOdom::addImu(ImuData& source_imu)
{
    //
}

void LidarOdom::addPointcloud(pcl::PointCloud<pcl::PointXYZ>& source_cloud)
{
    cout << "Received a pointcloud. [" 
    << common::GetSecFromMicro(source_cloud.header.stamp) << "."
    << common::GetNanoFromMicro(source_cloud.header.stamp) << "]" << endl;
}