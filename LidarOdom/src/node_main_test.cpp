/*
描述：尝试应用ImuTracker
作者：王冠华
日期：20210223
*/

#include <iostream>
// #include <fstream>
// #include <sstream>
// #include <algorithm>
#include <chrono> // for c++ time
#include <ratio> // for c++ time
#include <math.h>

#include<ros/ros.h>
#include <tf/transform_broadcaster.h>
#include<std_msgs/String.h>
// #include<std_msgs/Time.h>
// #include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/PointCloud.h>
#include<sensor_msgs/point_cloud_conversion.h>
// #include<message_filters/subscriber.h>
// #include<message_filters/time_synchronizer.h>
// #include<message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "liv_time.h"
#include "imu_tracker.h"
#include "DynamicObjFilter.h"

using namespace std;

class ImuReceiver{
  public:
    ImuReceiver(ros::Publisher& pub);
    ~ImuReceiver();
    void HandleImuMessage(const sensor_msgs::Imu::ConstPtr& imu_msg);
  private:
    // ros::NodeHandle& nh_;
    // ros::Subscriber& sub_;
    ros::Publisher& pub_;
    std::size_t num_imu_data_;
    ::cartographer::mapping::ImuTracker imu_tracker_;
};

// wgh-- 声明变量&对象
::cartographer::mapping::ImuTracker imu_tracker_;
std::size_t num_imu_data_;
::cartographer::mapping::DynamicObjFilter dynamic_obj_filter_(5.0, 0.6, 0.6, 0.6, 0.6, true);
// wgh-- 声明回调函数
void HandleImuMessage(const sensor_msgs::Imu::ConstPtr& imu_msg);
void HandlePointCloud2Message(const sensor_msgs::PointCloud2::ConstPtr& msg);
void TestVectorIterator();
bool IsGreater(double input_dist){return input_dist>5.0; };
bool IsLess(double input_dist){return input_dist<0.4; };
void TestEigenRotation();
void TestEigenIsometry();

int main(int argc, char **argv)
{
    if(argc > 3)
    {
        ROS_ERROR_STREAM("Input params too much!");
        return 1;
    }

    ros::init(argc, argv, "imu_tracker_node");
    ros::start();
    ros::NodeHandle nh;

    // ros::Publisher pub = nh.advertise<>("/imu_frame", 100);
    // ImuReceiver imu_receiver_(pub);
    // ros::Subscriber sub = nh.subscribe("/imu/data", 100, &ImuReceiver::HandleImuMessage, &imu_receiver_);

    TestVectorIterator();
    // TestEigenRotation();
    // TestEigenIsometry();
    ::cartographer::mapping::Rigid3d pose_test;
    pose_test.position = Eigen::Vector3d(1, 1, 1);
    pose_test.oritation = Eigen::Quaterniond(1, 1, 1, 1);
    pose_test.oritation = pose_test.oritation.normalized();
    dynamic_obj_filter_.AddPose(pose_test);

    num_imu_data_ = 0;
    // ros::Subscriber sub = nh.subscribe("/imu/data", 100, &HandleImuMessage);
    ros::Subscriber sub2 = nh.subscribe("/rslidar_points", 100, &HandlePointCloud2Message);

    ros::spin();

    ros::shutdown(); // wgh-- 与ros::start();成对出现
    return 0;
}

ImuReceiver::ImuReceiver(ros::Publisher& pub):
    pub_(pub), num_imu_data_(0), imu_tracker_()
{}

void ImuReceiver::HandleImuMessage(const sensor_msgs::Imu::ConstPtr& imu_msg){
    ++num_imu_data_;
    ROS_INFO("call back ImuReceiver::HandleImuMessage ...");
    //校验imu数据有效性
    //转化格式
    constexpr std::int64_t EpochOffsetInSeconds = (719162ll * 24ll * 60ll * 60ll);
    ::cartographer::common::Time imu_time_ = ::cartographer::common::FromUniversal(
        (imu_msg->header.stamp.sec + EpochOffsetInSeconds)*10000000ll+(imu_msg->header.stamp.nsec+50)/100
    );

    // Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    // Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    ::cartographer::ImuData imu_data_{
        imu_time_,
        Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z),
        Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z)
    };

    //调用算法
    imu_tracker_.Process(imu_data_);
    
    //写计时程序
    //广播world与imu_frame之间的tf数据
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    Eigen::Quaterniond imu_orientation = imu_tracker_.orientation();
	tf::Quaternion q( imu_orientation.x(), imu_orientation.y(), imu_orientation.z(), imu_orientation.w() );
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, imu_msg->header.stamp, "world", "imu_frame"));
    return;
}

void HandleImuMessage(const sensor_msgs::Imu::ConstPtr& imu_msg){
    ++num_imu_data_;
    // ROS_INFO("call back HandleImuMessage ...");
    std::cout << "call back HandleImuMessage #" << num_imu_data_ << std::endl ;
    // ROS_INFO("call back HandleImuMessage #d%", num_imu_data_);
    //校验imu数据有效性
    //转化格式
    constexpr std::int64_t EpochOffsetInSeconds = (719162ll * 24ll * 60ll * 60ll);
    ::cartographer::common::Time imu_time_ = ::cartographer::common::FromUniversal(
        (imu_msg->header.stamp.sec + EpochOffsetInSeconds)*10000000ll+(imu_msg->header.stamp.nsec+50)/100
    );

    // Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    // Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    ::cartographer::ImuData imu_data_{
        imu_time_,
        Eigen::Vector3d(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z),
        Eigen::Vector3d(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z)
    };

    //调用算法
    imu_tracker_.ProcessWithGravity(imu_data_);
    
    //写计时程序
    //广播world与imu_frame之间的tf数据
	static tf::TransformBroadcaster br;
	tf::Transform transform;
    // Eigen::Vector3d const imu_position = imu_tracker_.position();
    // tf::Vector3 imu_position_(imu_position.x(), imu_position.y(), imu_position.z());
    // std::cout << "tf::Vector3 imu_position_: " << imu_position_.x() << " " << imu_position_.y() << " " << imu_position_.z() << std::endl;
    // transform.setOrigin( imu_position_ );
    // transform.setOrigin( tf::Vector3( imu_position.x(), imu_position.y(), imu_position.z() ) );
	transform.setOrigin( tf::Vector3(.0, .0, .2) );
    Eigen::Quaterniond imu_orientation = imu_tracker_.orientation();
	tf::Quaternion q( imu_orientation.x(), imu_orientation.y(), imu_orientation.z(), imu_orientation.w() );
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, imu_msg->header.stamp, "world", "imu_frame"));
    return;
}


void HandlePointCloud2Message(const sensor_msgs::PointCloud2::ConstPtr& msg){
    std::cout << "msg->header.frame_id: " << msg->header.frame_id << std::endl;
    // wgh-- 去除frame id中的“/”（如有）
    std::string frame_id_ = msg->header.frame_id;
    if(frame_id_.front() == '/'){
        frame_id_.erase( std::remove(frame_id_.begin(), frame_id_.end(), '/'), frame_id_.end() );
        std::cout << "msg->header.frame_id is modified as: " << frame_id_ << std::endl;
    }

    sensor_msgs::PointCloud out_pointcloud;
	sensor_msgs::convertPointCloud2ToPointCloud(*msg, out_pointcloud);
	for (int i=0; i<out_pointcloud.points.size(); i++) {
		// std::cout << out_pointcloud.points[i].x << ", " << out_pointcloud.points[i].y << ", " 
        //     << out_pointcloud.points[i].z<< "------" << std::endl;
	}
    return;
}












    // // wgh-- customed part
    // std::chrono::time_point<std::chrono::system_clock> now =
    //     std::chrono::system_clock::now();
    // std::vector<std::chrono::time_point<std::chrono::system_clock>> times {
    //     now - std::chrono::hours(24),
    //     now - std::chrono::hours(48),
    //     now,
    //     now + std::chrono::hours(24),
    // };
    // std::chrono::time_point<std::chrono::system_clock> time_of_max =
    //     std::chrono::time_point<std::chrono::system_clock>::max();
    
    // ROS_INFO_STREAM("Hello ROS");
    // ROS_WARN_STREAM("all times:\n");
    // ROS_INFO("can U see this node started?");
    // ROS_INFO("all times:\n");
    // for (const auto &time : times) {
    //     std::time_t t = std::chrono::system_clock::to_time_t(time);
    //     ROS_INFO_STREAM(std::ctime(&t));
    //     // std::cout << std::ctime(&t);
    // }
    // std::time_t t = std::chrono::system_clock::to_time_t(time_of_max);
    // ROS_INFO_STREAM(std::ctime(&t));

void TestVectorIterator(){
    // ************************************** begin *************************************
    const uint32_t loop = 1000000;
    std::vector<int32_t> vec;
    clock_t timeStart = 0;
    for (uint32_t i = 0; i < loop; ++i)
    {
        vec.push_back(i);
    }
    // test time use
    // 1.by index
    timeStart = clock();
    for (uint32_t i = 0; i < vec.size(); ++i)
    {
        vec[i]++;
        vec[i]--;
    }
    std::cout << (clock() - timeStart)/1000.0f << "ms" << std::endl;
    // 2.by iterator
    timeStart = clock();
    for (std::vector<int32_t>::const_iterator it = vec.begin(); it != vec.end(); ++it)
    {
        *it++;
        *it--;
    }
    std::cout << (clock() - timeStart)/1000.0f << "ms" << std::endl;
    // 3.by auto iterator
    timeStart = clock();
    for (auto it = vec.begin(); it != vec.end(); ++it)
    {
        *it++;
        *it--;
    }
    std::cout << (clock() - timeStart)/1000.0f << "ms" << std::endl;
    // ************************************** -end- *************************************
    // ********* another test begin
    std::vector<double> dists;
    dists.push_back(4.15);
    dists.push_back(0.39);
    dists.push_back(5.00);
    dists.push_back(5.12);
    dists.push_back(4.99);
    dists.push_back(5.01);
    dists.push_back(5.03);
    dists.push_back(289.0);
    dists.push_back(0.002);
    dists.push_back(0.6);
    dists.push_back(5.002);
    dists.push_back(9.7);
    dists.erase(remove_if(dists.begin(), dists.end(), IsGreater), dists.end());
    std::cout << "dists.size(): " << dists.size() << std::endl;
    std::cout << "we print dists after erase-remove_if operation: " << std::endl;
    for(std::size_t i=0; i< dists.size(); i++){
        std::cout << dists[i] << std::endl;
    }
    std::cout << "we test reverse iterator of vector here: " << std::endl;
    for(auto it = dists.rbegin(); it != dists.rend(); ++it){
        std::cout << *it << std::endl;
    }

    // 测试排序
    sort(dists.rbegin(), dists.rend());
    std::cout << "we print dists after sort_reverse operation: " << std::endl;
    for(std::size_t i=0; i< dists.size(); i++){
        std::cout << dists[i] << std::endl;
    }
    // 移除2,4,5位置的元素（第3,5,6个）
    std::cout << "going to test [removal_index]: " << std::endl;
    std::vector<std::size_t> removal_index;
    removal_index.push_back(1); removal_index.push_back(3); removal_index.push_back(5);
    for(std::size_t j=removal_index.size()-1; j>=0; --j){
        // int diff = removal_index[j];
        std::cout << "here" << removal_index[j] << std::endl;
        dists.erase(dists.begin()+removal_index[j]);
        if(j==0){
            break;
        }
    }
    std::cout << "we print dists after [removal_index]: " << std::endl;
    for(std::size_t i=0; i< dists.size(); i++){
        std::cout << dists[i] << std::endl;
    }
    // ********* another test end
    return;
}


void TestEigenRotation(){
    Eigen::AngleAxisd rotat_vec(-M_PI/4, Eigen::Vector3d(sqrt(1.0/3.0), sqrt(1.0/3.0), sqrt(1.0/3.0)));
    Eigen::Quaterniond rotat_quat(rotat_vec);
    rotat_vec = rotat_quat.normalized();
    cout << "rotat_vec angle & axis: " << rotat_vec.angle()/M_PI*180.0 << " ; "<< rotat_vec.axis().transpose() << endl;
    Eigen::Vector3d euler_angles = rotat_vec.toRotationMatrix().eulerAngles(2, 1, 0);
    cout << "yaw pitch roll: " << euler_angles.transpose()/M_PI*180.0 << endl;
    euler_angles = rotat_vec.toRotationMatrix().eulerAngles(0, 1, 2);
    cout << "roll pitch yaw: " << euler_angles.transpose()/M_PI*180.0 << endl;
    /* 关于转换为欧拉角：实验证明，转换顺序不同，角度值不同！ */
    // ------------
    Eigen::Vector3d orig_point(0, 0, 1);
    Eigen::Vector3d point1 = rotat_vec.inverse().toRotationMatrix() * orig_point;
    cout << "orig_point                            : " << orig_point.transpose() << endl;
    cout << "orig_point after roration (body frame): " << point1.transpose() << endl;
    point1 = Eigen::Vector3d(0, 0, 1);
    orig_point = rotat_vec.toRotationMatrix() * point1;
    cout << "point1 in body frame               : " << point1.transpose() << endl;
    cout << "point1 after roration (world frame): " << orig_point.transpose() << endl;
}


void TestEigenIsometry(){
    /* 测试齐次变换 */
    // Eigen::AngleAxisd rotat_vec(-M_PI/4, Eigen::Vector3d(sqrt(1.0/3.0), sqrt(1.0/3.0), sqrt(1.0/3.0)));
    Eigen::AngleAxisd rotat_vec2(-M_PI/4.0, Eigen::Vector3d(0, 1, 0));
    Eigen::AngleAxisd rotat_vec(M_PI/3.0, Eigen::Vector3d(0, 0, 1));
    cout << "rotat_vec angle & axis: " << rotat_vec.angle()/M_PI*180.0 << " ; "<< rotat_vec.axis().transpose() << endl;
    Eigen::Vector3d euler_angles = rotat_vec.toRotationMatrix().eulerAngles(2, 1, 0);
    cout << "yaw pitch roll: " << euler_angles.transpose()/M_PI*180.0 << endl;
    // ------------
    std::cout << " ----------------- splitter ----------------- " << std::endl;
    Eigen::Vector3d space_point(0, 0, 1);
    Eigen::Vector3d body_orig_point(1, 0, 1);
    Eigen::Isometry3d trans_ = Eigen::Isometry3d::Identity();
    /* 注意：此处，旋转/平移的先后顺序有影响。 */
    trans_.rotate(rotat_vec);
    trans_.pretranslate(body_orig_point);
    // trans_.prerotate(rotat_vec);
    // trans_.translate(body_orig_point);
    cout << "space_point in world frame: " << space_point.transpose() << endl;
    space_point = trans_.inverse() * space_point;
    cout << "space_point in body  frame: " << space_point.transpose() << endl;
}