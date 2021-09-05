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
#include<pcl/filters/radius_outlier_removal.h>
// OpenCV
#include<opencv/cv.hpp>
// 自定义
#include"LidarOdom.h"
#include"liv_time.h"

void getCloudOrganized(pcl::PointCloud<PointType>& input_cloud)
{
    //
}


LidarOdom::LidarOdom(LidarOdomOptions& lo_options)
    :lo_options_(lo_options)
{
    globalMapDS.reset(new pcl::PointCloud<PointType>);

    newCloudOriginal.reset(new pcl::PointCloud<PointType>);
    currCloudOriginal.reset(new pcl::PointCloud<PointType>);
    currCloudFiltered.reset(new pcl::PointCloud<PointType>);
    currCloudDynamicd.reset(new pcl::PointCloud<PointType>);
    lastCloudOriginal.reset(new pcl::PointCloud<PointType>);
    lastCloudFiltered.reset(new pcl::PointCloud<PointType>);

    currRangeImg = cv::Mat::zeros(16, 1800, CV_32FC1); /* Mat初始化.[required] */
}

LidarOdom::~LidarOdom()
{
    //
}

void LidarOdom::addImu(const ImuData& source_imu)
{
    imu_tracker_.addImu(source_imu);
}

void LidarOdom::addPointcloud(const common::Time& time_stamp, const pcl::PointCloud<PointType>& source_cloud)
{
    num_received_cloud++;
    cout << "Received a pointcloud #" << num_received_cloud << ". ["
    << common::GetSecFromMicro(source_cloud.header.stamp) << "."
    << common::GetNanoFromMicro(source_cloud.header.stamp) << "]" << endl << endl;
    // We dont actually use time stamp from PCL point cloud header, 
    // since it was converted to microsecond, which may cause accuracy loss. 

    // /* step# 保留new msg的必要信息 */
    // // newCloudOriginal
    // /* step# 重置变量 */
    // // globalMapDS->clear();
    // currCloudOriginal->clear();
    // currCloudFiltered->clear();
    // currCloudDynamicd->clear();
    // *currCloudOriginal = source_cloud;
    // /* step# 点云预处理 */
    // // pcl::RadiusOutlierRemoval<PointType> outlier_filter;
    // // outlier_filter.setRadiusSearch(0.3);
    // // outlier_filter.setMinNeighborsInRadius(1);
    // // outlier_filter.setInputCloud(currCloudOriginal);
    // // std::vector<int> indices;
    // // outlier_filter.filter(indices);

    /* step# 第1帧无需配准，保留必要信息，退出。 */
    if (num_received_cloud==1) {
        Eigen::Vector3d gravity_temp = imu_tracker_.getGravity(time_stamp);
        Eigen::Quaterniond orient_temp = Eigen::Quaterniond::Identity();
        const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
            -gravity_temp, orient_temp.conjugate() * Eigen::Vector3d::UnitZ());
        orient_temp = (orient_temp * rotation).normalized();
        *lastCloudOriginal = source_cloud;
        lastPose.id = 0;
        lastPose.time = time_stamp;
        lastPose.pose = Rigid3d(Eigen::Vector3d(0,0,0),orient_temp);
        lastPose.gravity = gravity_temp;
        return;
    }

    /* step# 算法延迟一帧处理，保留第2帧的必要信息、第1帧deskew并存入容器、退出。 */
    if (num_received_cloud==2) {
        *currCloudOriginal = source_cloud;
        currPose.time = time_stamp;
        currPose.gravity = imu_tracker_.getGravity(time_stamp);

        // TODO: deskew the 1st point cloud.
        // for (size_t i=0; i<lastCloudOriginal)
        for (size_t i=0; i<lastCloudOriginal->size(); i++)
        {
            //
        }
        // TODO: save the 1st point cloud and it's pose in containers.
        // 思考一个问题，容器的元素类型是点云指针，这个指针肯定不能是后边会被reset的指针。
        // 等等，push_back时，指针变量本身被复制了(浅拷贝)，那就不用担心了呀！

        return;
    }

    /* step#1 用Imu tracker 处理两帧之间的imu数据 */
    /* step#2  */
    /* step#3  */

    //设置使用机器人测距法得到的初始对准估计结果
    Eigen::AngleAxisf init_rotation (0.0, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation (0.0, 0.0, 0.0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

    // 只有特征点前端才需要用这个，暂时屏蔽
    // 对点云进行organized
    currRangeImg = cv::Mat::zeros(16, 1800, CV_32FC1); /* Mat必须初始化为0像素值！ */
    f_extractor_.getRangeImage(source_cloud, currRangeImg);
}


// #######################################################################
// ******************************** 分界线 ********************************
// #######################################################################

void LidarOdom::CalculateCloudInfo(pcl::PointCloud<PointType>::Ptr& input_cloud_)
{
    // TODO: 如果点云自带有效时间戳了，就只计算每个点的laserID，保存到intensity中；
    // TODO: 否则，计算相对第一个点的时间戳，并计算laserID。

    return;
}


void LidarOdom::ndtMatch(pcl::PointCloud<PointType>::Ptr target_cloud,
            pcl::PointCloud<PointType>::Ptr input_cloud,
            const Eigen::Matrix4f& init_guess,
            Eigen::Matrix4f& pose_estimate)
{
    //将输入的扫描过滤到原始尺寸的大概10%以提高匹配的速度
    pcl::PointCloud<PointType>::Ptr filtered_input (new pcl::PointCloud<PointType>);
    pcl::ApproximateVoxelGrid<PointType> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud (input_cloud);
    approximate_voxel_filter.filter (*filtered_input);
    std::cout << "Filter input cloud from " << input_cloud->size() << " points to "
            << filtered_input->size() << " points" << std::endl;
    //初始化正态分布变换（NDT）
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
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
    ndt.setInputSource(filtered_input);
    //设置点云配准目标
    ndt.setInputTarget (target_cloud);
    //计算需要的刚体变换以便将输入的点云匹配到目标点云
    pcl::PointCloud<PointType>::Ptr output_cloud (new pcl::PointCloud<PointType>);
    ndt.align (*output_cloud, init_guess);
    cout << "NDT has converged:" << ndt.hasConverged()
            << " score: " << ndt.getFitnessScore() << endl;
    //使用创建的变换对未过滤的输入点云进行变换
    pose_estimate = ndt.getFinalTransformation();
    pcl::transformPointCloud (*input_cloud, *output_cloud, pose_estimate);
    // pose_estimate.
    return;
}


void LidarOdom::featureMatch()
{
    //
    return;
}


// Eigen::Vector3d LidarOdom::getLastTranslation()
// {
//     if (filteredCloudKeyPoses.size()<2) return Eigen::Vector3d::Zero();
//     std::size_t num_poses = filteredCloudKeyPoses.size();
//     return filteredCloudKeyPoses.back().pose.translation() - 
//         filteredCloudKeyPoses[num_poses-2].pose.translation();
// }