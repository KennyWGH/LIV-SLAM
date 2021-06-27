/**
 * 描述...
 * 
 */


// ROS
#include<pcl_conversions/pcl_conversions.h>
// 自定义工程
#include"LidarOdomWrapper.h"


LidarOdomWrapper::LidarOdomWrapper(ros::NodeHandle& nh, Options& options)
    :nh_(nh), options_(options), lidar_odom_(options.lidar_odom_options), imu_warn_sampler(0.01)
{
    // 订阅话题
    imu_sub = nh_.subscribe(options_.imu_topic, 1000, &LidarOdomWrapper::addImu, this);
    pointcloud_sub = nh_.subscribe(options_.pointcloud_topic, 100, &LidarOdomWrapper::addPointcloud, this);
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
            ROS_INFO_STREAM("Dispatch to core an IMU data.        -- [" 
                << front_imu->header.stamp.sec << "." << front_imu->header.stamp.nsec << "]");
            // return; /* this can be removed, so we can dispatch one imu and one pc2, instead of just one imu. */
        }
    }
    // 数据队列已初始化成功，正常按时间顺序下发数据
    if(imu_queue_.front()->header.stamp.toSec()<pointcloud_queue_.front()->header.stamp.toSec())
    {
        sensor_msgs::ImuConstPtr ros_imu = imu_queue_.front();
        ROS_INFO_STREAM("Dispatch to core an IMU data. [" << ros_imu->header.stamp.sec
            << "." << ros_imu->header.stamp.nsec << "]");
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