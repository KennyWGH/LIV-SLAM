/**
 * 描述...
 * 
 */


// ROS
#include<pcl_conversions/pcl_conversions.h>
// OpenCV
#include<opencv/cv.hpp>
// 自定义工程
#include"LidarOdomWrapper.h"


LidarOdomWrapper::LidarOdomWrapper(ros::NodeHandle& nh, Options& options)
    :nh_(nh), options_(options), lidar_odom_(options.lidar_odom_options), imu_warn_sampler(0.01)
{
    // 订阅话题
    subImu = nh_.subscribe(options_.imu_topic, 1000, &LidarOdomWrapper::addImu, this);
    subPointcloud = nh_.subscribe(options_.pointcloud_topic, 100, &LidarOdomWrapper::addPointcloud, this);
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
    return;
}


// 下发数据的核心函数，负责数据队列初始化和按时间顺序下发。
// 由于队列中数据个数不少于2个时才会下发，因此会有一帧点云的延迟。
// 已优化！延迟很小：约20ms！
void LidarOdomWrapper::checkAndDispatch()
{
    if(!(imu_queue_.size()>1&&pointcloud_queue_.size()>=1)) return;
    if(!isInitialized){
        /* 情形一：所有imu都在第一帧点云之后：弹出第一个imu之前的所有点云 */
        if( pointcloud_queue_.front()->header.stamp.toNSec()<imu_queue_.front()->header.stamp.toNSec() )
        {
            do{
                pointcloud_queue_.pop();
            }while(!pointcloud_queue_.empty() 
                && pointcloud_queue_.front()->header.stamp.toNSec()<imu_queue_.front()->header.stamp.toNSec());
            return;
        /* 情形二：所有imu都在第一帧点云之前：则只保留最近的一个imu */
        }else if(imu_queue_.back()->header.stamp.toNSec()<pointcloud_queue_.front()->header.stamp.toNSec()){
            while(imu_queue_.size()>1)
            {
                imu_queue_.pop();
            }
            return;
        /* 情形三：第一帧点云前后都有imu，符合条件，队列初始化完成，开始下发 */
        }else{
            isInitialized=true;
            sensor_msgs::ImuConstPtr front_imu;
            do{
                front_imu = imu_queue_.front();
                imu_queue_.pop();
            }while(imu_queue_.front()->header.stamp.toNSec()<pointcloud_queue_.front()->header.stamp.toNSec());
            std::size_t num = imu_queue_.size() + pointcloud_queue_.size();
            ROS_INFO_STREAM("Data queue initialized! Remaining queue size: " 
                << imu_queue_.size() << " + " << pointcloud_queue_.size() << " = " << num);
            ROS_INFO_STREAM("Dispatch to core the first IMU data.        -- [" 
                << front_imu->header.stamp.sec << "." << front_imu->header.stamp.nsec << "]");
            /* `return` can be removed, so we can dispatch one imu and one pc2, instead of just one imu. */
            // return; 
        }
    }
    // 数据队列已初始化成功，正常按时间顺序下发数据
    if(imu_queue_.front()->header.stamp.toSec()<pointcloud_queue_.front()->header.stamp.toSec())
    {
        sensor_msgs::ImuConstPtr ros_imu = imu_queue_.front();
        // ROS_INFO_STREAM("Dispatch to core an IMU data. [" << ros_imu->header.stamp.sec
        //     << "." << ros_imu->header.stamp.nsec << "]");
        ImuData imu_data {common::Time( common::FromSeconds(double(ros_imu->header.stamp.sec))+
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
    sensor_msgs::PointCloud2ConstPtr ros_pc2 = pointcloud_queue_.front();
    ROS_INFO_STREAM("Dispatch to core a Pointcloud data.     [" << ros_pc2->header.stamp.sec
        << "." << ros_pc2->header.stamp.nsec <<"]");
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*ros_pc2, pcl_cloud);
    lidar_odom_.addPointcloud(pcl_cloud);
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
        ROS_INFO_STREAM("average depth from intensity is: " << temp_intensity_avg);
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