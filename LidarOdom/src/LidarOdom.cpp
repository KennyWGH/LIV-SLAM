/**
 * 描述
 * 
 * 
 * 
 */


// C++
#include<iostream>
// PCL
#include<pcl/registration/ndt.h>
#include<pcl/filters/approximate_voxel_grid.h>
// OpenCV
#include<opencv/cv.hpp>
// 自定义
#include"LidarOdom.h"
#include"liv_time.h"

void getCloudOrganized(pcl::PointCloud<pcl::PointXYZ>& input_cloud)
{
    //
}


LidarOdom::LidarOdom(LidarOdomOptions& lo_options)
    :lo_options_(lo_options)
{
    //
}

LidarOdom::~LidarOdom()
{
    //
}

void LidarOdom::addImu(const ImuData& source_imu)
{
    //
}

void LidarOdom::addPointcloud(const pcl::PointCloud<pcl::PointXYZ>& source_cloud)
{
    // cout << "Received a pointcloud. [" 
    // << common::GetSecFromMicro(source_cloud.header.stamp) << "."
    // << common::GetNanoFromMicro(source_cloud.header.stamp) << "]" << endl;

    //设置使用机器人测距法得到的初始对准估计结果
    Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

    // 对点云进行organized
    cur_range_img = cv::Mat::zeros(16, 1800, CV_32FC1); /* Mat必须初始化为0像素值！ */
    f_extractor_.getRangeImage(source_cloud, cur_range_img);
}


void LidarOdom::ndtMatch(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
            const Eigen::Matrix4f& init_guess,
            Eigen::Matrix4f& pose_estimate)
{
    //将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_input (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (input_cloud);
    approximate_voxel_filter.filter (*filtered_input);
    std::cout << "Filter input cloud from " << input_cloud->size() << " points to "
            << filtered_input->size() << " points" << std::endl;
    //初始化正态分布变换（NDT）
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    //设置依赖尺度NDT参数
    //为终止条件设置最小转换差异
    ndt.setTransformationEpsilon (0.01);
    //为More-Thuente线搜索设置最大步长
    ndt.setStepSize (0.1);
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt.setResolution (1.0);
    //设置匹配迭代的最大次数
    ndt.setMaximumIterations (35);
    // 设置要配准的点云
    ndt.setInputCloud (filtered_input);
    //设置点云配准目标
    ndt.setInputTarget (target_cloud);
    //计算需要的刚体变换以便将输入的点云匹配到目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align (*output_cloud, init_guess);
    cout << "NDT has converged:" << ndt.hasConverged()
            << " score: " << ndt.getFitnessScore() << endl;
    //使用创建的变换对未过滤的输入点云进行变换
    pose_estimate = ndt.getFinalTransformation();
    // pcl::transformPointCloud (*input_cloud, *output_cloud, pose_estimate);
    return;
}


void LidarOdom::featureMatch()
{
    //
    return;
}