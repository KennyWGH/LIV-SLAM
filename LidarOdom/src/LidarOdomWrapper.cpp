/**
 * 描述...
 * 
 */

// MUST define this if you are using custom point-type along with PCL algorithms.
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

// ROS
#include<pcl_conversions/pcl_conversions.h>
// PCL
#include<pcl/filters/radius_outlier_removal.h>
#include<pcl/filters/impl/radius_outlier_removal.hpp>
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/impl/voxel_grid.hpp>
// OpenCV
#include<opencv/cv.hpp>
// 自定义工程
#include"LidarOdomWrapper.h"


LidarOdomWrapper::LidarOdomWrapper(ros::NodeHandle& nh, Options& options)
    :nh_(nh), options_(options), 
    lidar_odom_(options.lidar_odom_options), 
    imu_warn_sampler(0.01)
{
    // 订阅话题
    subImu = nh_.subscribe(options_.imu_topic, 1000, &LidarOdomWrapper::addImu, this);
    subPointcloud = nh_.subscribe(options_.pointcloud_topic, 100, 
                                    &LidarOdomWrapper::addPointcloud, this);
    pubRangeImg = nh_.advertise<sensor_msgs::Image>("range_img", 1);
    pubTempCloud = nh_.advertise<sensor_msgs::PointCloud2>("temp_cloud", 1);
    // 其它
    show_cvImg_sampler.setRatio(0.1);
}


LidarOdomWrapper::~LidarOdomWrapper() {}


void LidarOdomWrapper::addImu(const sensor_msgs::ImuConstPtr& msg)
{
    if(imu_queue_.empty()){
        imu_queue_.push(msg);
        return;
    }
    const auto msg_time = msg->header.stamp.toNSec();
    const auto queue_newest_time = imu_queue_.back()->header.stamp.toNSec();
    if(!msg_time>queue_newest_time)
    {
        if(imu_warn_sampler.pulse()) ROS_WARN_STREAM("IMU data too old! Drop it ... ");
        return;
    }
    imu_queue_.push(msg);
    trimImuQueue();
    checkAndDispatch();
    return;
}


void LidarOdomWrapper::addPointcloud(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if(pointcloud_queue_.empty()){
        pointcloud_queue_.push(msg);
        return;
    }
    const auto msg_time = msg->header.stamp.toNSec();
    const auto queue_newest_time = pointcloud_queue_.back()->header.stamp.toNSec();
    if(!msg_time>queue_newest_time)
    {
        ROS_WARN_STREAM("Pointcloud data too old! Drop it ... ");
        return;
    }
    pointcloud_queue_.push(msg);
    trimPointcloudQueue();
    checkAndDispatch();
    // std::cout << std::endl << std::endl;
    return;
}


// 如果既有ring又有time，则返回true，否则返回false。
bool LidarOdomWrapper::tryConvertROSCloudToRingTimeCloud(
        sensor_msgs::PointCloud2ConstPtr& ros_cloud, 
        pcl::PointCloud<PointType>& out_cloud)
{
    // check ROS msg fields
    // for VLP-16 ROS driver, {x/y/z/intensity/ring/time} are included.
    bool hasRing = false;
    bool hasTime = false;
    for (std::size_t i=0; i<ros_cloud->fields.size(); i++) {
        // std::cout << "ROS cloud msg: #" << i << " " << ros_cloud->fields[i].name << std::endl;
        if (ros_cloud->fields[i].name=="ring") hasRing=true;
        if (ros_cloud->fields[i].name=="time") hasTime=true;
    }

    pcl::PointCloud<PointXYZIRT> temp_cloud;
    pcl::fromROSMsg(*ros_cloud, temp_cloud);

    // let's do a test to verify whether custom pointcloud structure can work with PCL algorithms.
    if (0)
    {
        pcl::PointCloud<PointXYZIRT>::Ptr temp_cloud_ptr;
        temp_cloud_ptr.reset(new pcl::PointCloud<PointXYZIRT>);
        *temp_cloud_ptr = temp_cloud;
        std::size_t size_before = temp_cloud_ptr->points.size();
        pcl::VoxelGrid<PointXYZIRT> downSizeFilter;
        downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
        downSizeFilter.setInputCloud(temp_cloud_ptr);
        downSizeFilter.filter(*temp_cloud_ptr);
        std::cout << "filtered points size is " << temp_cloud_ptr->points.size() 
                    << " of " << size_before << std::endl;
    }

    // do a test to verify whether `clear()` will weep out pcl cloud header.
    if (0) 
    {
        pcl::fromROSMsg(*ros_cloud, out_cloud);
        out_cloud.is_dense = 100;
        out_cloud.width = 6849;
        std::cout << "cloud msg: dense " << out_cloud.is_dense
                << ", width " << out_cloud.width << ", heigth " << out_cloud.height 
                << ", header " << out_cloud.header 
                << ", cloud size " << out_cloud.points.size() << std::endl;
        out_cloud.clear();
        std::cout << "cloud msg: dense " << out_cloud.is_dense
                << ", width " << out_cloud.width << ", heigth " << out_cloud.height 
                << ", header " << out_cloud.header 
                << ", cloud size " << out_cloud.points.size() << std::endl;
    }

    out_cloud.clear();
    out_cloud.header = temp_cloud.header;
    out_cloud.is_dense = false; // we use this to indicate if Ring&Time is specified.

    if(hasRing && hasTime) {
        // double avg_time = 0;
        // double avg_ring = 0;
        // double totol_num = out_cloud.points.size();
        // double ring0_num = 0;
        // double timePositive_num = 0;
        // int factor = (int)(totol_num/20);
        // double time_local = 0;

        PointType thisPoint;
        for (std::size_t i=0; i<temp_cloud.points.size(); i++)
        {
            // avg_time += temp_cloud.points[i].time;
            // avg_ring += temp_cloud.points[i].ring;
            // float RingTime = 0.;
            // if (0<=temp_cloud.points[i].ring && temp_cloud.points[i].ring<=16
            //     && 0.<=temp_cloud.points[i].time && temp_cloud.points[i].time <1.0 )
            // {
            //     RingTime = temp_cloud.points[i].ring + temp_cloud.points[i].time;
            // }
            // out_cloud.points[i].intensity = RingTime;

            // // if (i<10) std::cout << "time first 10: " << temp_cloud.points[i].time << std::endl;
            // // if (totol_num-11<i) std::cout << "time last 10: " << temp_cloud.points[i].time << std::endl;
            // if (temp_cloud.points[i].ring==0 ) ring0_num++;
            // if (temp_cloud.points[i].time>0 ) timePositive_num++;

            // time_local+=temp_cloud.points[i].time;
            // if(i>0&&(i%factor)==0){
            //     std::cout << "local avg time " << time_local/factor << std::endl;
            //     time_local=0;
            // }

            if (0<=temp_cloud.points[i].ring && temp_cloud.points[i].ring<=16
                && -1.<temp_cloud.points[i].time && temp_cloud.points[i].time <0. )
            {
                thisPoint.x = temp_cloud.points[i].x;
                thisPoint.y = temp_cloud.points[i].y;
                thisPoint.z = temp_cloud.points[i].z;
                thisPoint.intensity = temp_cloud.points[i].ring + temp_cloud.points[i].time;
                out_cloud.push_back(thisPoint);
            }

        }
        // std::cout << "for this cloud msg, avg ring " << avg_ring/totol_num << ", avg time " << avg_time/totol_num 
        //     << "  |  ringID(0) " << ring0_num << ", " << ring0_num/totol_num*100 << "%  |  time positive " 
        //     << timePositive_num << ", " << timePositive_num/totol_num*100 << "%" << std::endl;
        
        out_cloud.is_dense = true;
        return true;
    }

    if(!hasRing && hasTime) {
        //
    }
    if(hasRing && !hasTime) {
        //
    }

    pcl::fromROSMsg(*ros_cloud, out_cloud);
    out_cloud.is_dense = false;
    return false;
    
}


// 下发数据的核心函数，负责数据队列初始化和按时间顺序下发。
// 由于队列中数据个数不少于2个时才会下发，因此会有一帧点云的延迟。
// 已优化！延迟很小：约20ms！
void LidarOdomWrapper::checkAndDispatch()
{
    if(!(imu_queue_.size()>1&&pointcloud_queue_.size()>=1)) return;
    if(!isInitialized){
        /* 情形一：所有imu都在第一帧点云之后：弹出第一个imu之前的所有点云 */
        if( pointcloud_queue_.front()->header.stamp.toNSec()
            <imu_queue_.front()->header.stamp.toNSec() )
        {
            do{
                pointcloud_queue_.pop();
            }while(!pointcloud_queue_.empty() 
                && pointcloud_queue_.front()->header.stamp.toNSec()
                    <imu_queue_.front()->header.stamp.toNSec());
            return;
        /* 情形二：所有imu都在第一帧点云之前：则只保留最近的一个imu */
        }else if(imu_queue_.back()->header.stamp.toNSec()
            <pointcloud_queue_.front()->header.stamp.toNSec())
        {
            while(imu_queue_.size()>1)
            {
                imu_queue_.pop();
            }
            return;
        /* 情形三：第一帧点云前后都有imu，符合条件，队列初始化完成，开始下发 */
        }else
        {
            isInitialized=true;
            sensor_msgs::ImuConstPtr front_imu;
            do{
                front_imu = imu_queue_.front();
                imu_queue_.pop();
            }while(imu_queue_.front()->header.stamp.toNSec()
                    <pointcloud_queue_.front()->header.stamp.toNSec());

            std::size_t num = imu_queue_.size() + pointcloud_queue_.size();
            ROS_INFO_STREAM("Data queue initialized! Remaining queue size: " 
                << imu_queue_.size() << " + " << pointcloud_queue_.size()-1 << " = " << num-1);
            ROS_INFO_STREAM("Dispatch to core the initial IMU data.        -- [" 
                << front_imu->header.stamp.sec << "." 
                << front_imu->header.stamp.nsec << "]");
            ImuData imu_data {  common::Time(
                                    common::FromSeconds(double(front_imu->header.stamp.sec))+
                                    common::FromNanoseconds(front_imu->header.stamp.nsec) ),
                                Eigen::Vector3d(front_imu->linear_acceleration.x,
                                                front_imu->linear_acceleration.y,
                                                front_imu->linear_acceleration.z),
                                Eigen::Vector3d(front_imu->angular_velocity.x,
                                                front_imu->angular_velocity.y,
                                                front_imu->angular_velocity.z) };
            lidar_odom_.addImu(imu_data);
            /* `return` can be removed, so we can dispatch one imu and one pc2 
            right after initialization, instead of just one imu. */
            // return; 
        }
    }

    // 数据队列已初始化成功，正常按时间顺序下发数据
    if(imu_queue_.front()->header.stamp.toSec()
        <pointcloud_queue_.front()->header.stamp.toSec())
    {
        sensor_msgs::ImuConstPtr ros_imu = imu_queue_.front();
        // ROS_INFO_STREAM("Dispatch to core an IMU data. [" << ros_imu->header.stamp.sec
        //     << "." << ros_imu->header.stamp.nsec << "]");
        ImuData imu_data {  common::Time(
                                common::FromSeconds(double(ros_imu->header.stamp.sec))+
                                common::FromNanoseconds(ros_imu->header.stamp.nsec) ),
                            Eigen::Vector3d(ros_imu->linear_acceleration.x,
                                            ros_imu->linear_acceleration.y,
                                            ros_imu->linear_acceleration.z),
                            Eigen::Vector3d(ros_imu->angular_velocity.x,
                                            ros_imu->angular_velocity.y,
                                            ros_imu->angular_velocity.z) };
        lidar_odom_.addImu(imu_data);
        imu_queue_.pop();
        return;
    }
    sensor_msgs::PointCloud2ConstPtr ros_cloud = pointcloud_queue_.front();
    // ROS_INFO_STREAM("Dispatch to core a Pointcloud data.     [" 
    //     << ros_cloud->header.stamp.sec << "." << ros_cloud->header.stamp.nsec <<"]");
    pcl::PointCloud<PointType> pcl_cloud;
    // pcl::fromROSMsg(*ros_cloud, pcl_cloud);
    tryConvertROSCloudToRingTimeCloud(ros_cloud, pcl_cloud);
    common::Time cloud_time_stamp = common::Time( 
                        common::FromSeconds(double(ros_cloud->header.stamp.sec))+
                        common::FromNanoseconds(ros_cloud->header.stamp.nsec) );
    lidar_odom_.addPointcloud(cloud_time_stamp, pcl_cloud);
    pointcloud_queue_.pop();

    /* 在RVIZ可视化深度图 */
    {
        cv::Mat img_shown;
        cv::resize(lidar_odom_.getCurRangeImg(),img_shown, cv::Size(180, 16));
        double temp_intensity_sum=0;
        /* 把range*2,这样100米对应最大像素值200<255，则最远可显示127.5米的距离 */
        /* 把range*10,这样25.5米对应最大像素值255，则最远可显示25.5米的距离 */
        /* 把range*20,这样12.75米对应最大像素值255，则最远可显示12.75米的距离 */
        for (int i=0; i<img_shown.rows; i++) {
            for (int j=0; j<img_shown.cols; j++) {
                temp_intensity_sum+=img_shown.at<float>(i,j);
                img_shown.at<float>(i,j)=img_shown.at<float>(i,j)*10;
            }
        }
        double temp_intensity_avg = temp_intensity_sum/(img_shown.rows*img_shown.cols);
        // ROS_INFO_STREAM("average depth from intensity is: " << temp_intensity_avg);
        img_shown.convertTo(img_shown, CV_8UC1); //COLOR_GRAY2BGR 
        cv::applyColorMap(img_shown, img_shown, cv::COLORMAP_RAINBOW);
        cv::flip(img_shown, img_shown, 0);
        sensor_msgs::ImagePtr img_msg = 
            cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_shown).toImageMsg();
            // cv_bridge::CvImage(std_msgs::Header(), "mono8", img_shown).toImageMsg();
        pubRangeImg.publish(*img_msg);
    }
    return;
}


// 队列最多只保存5秒的数据
void LidarOdomWrapper::trimImuQueue()
{
    bool doLoop = true;
    const auto newest_time = imu_queue_.back()->header.stamp.toSec();
    while(doLoop){
        if(imu_queue_.front()->header.stamp.toSec()+5.0<newest_time){
            imu_queue_.pop();
        }else{
            doLoop = false;
        }
    }
    return;
}


// 队列最多只保存5秒的数据
void LidarOdomWrapper::trimPointcloudQueue()
{
    bool doLoop = true;
    const auto newest_time = pointcloud_queue_.back()->header.stamp.toSec();
    while(doLoop){
        if(pointcloud_queue_.front()->header.stamp.toSec()+5.0<newest_time){
            pointcloud_queue_.pop();
        }else{
            doLoop = false;
        }
    }
    return;
}