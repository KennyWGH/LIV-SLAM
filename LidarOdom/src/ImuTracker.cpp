/**
 * 
 * 
 */

// C++
#include <cmath>
#include <limits>
#include <iostream>
// 自定义
#include "ImuTracker.h"




ImuTracker::ImuTracker(double imu_gravity_time_constant)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      gravity_constant_(9.806),
      processed_time_(common::Time::min()),
      curr_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(-Eigen::Vector3d::UnitZ()),
      last_angular_velocity_(Eigen::Vector3d::Zero()),
      last_linear_accel_corrected_(Eigen::Vector3d::Zero())
{
    std::cout << "ImuTracker initial position: " << position_.x() 
                << " " << position_.y() << " " << position_.z() << std::endl;
    std::cout << " -------------- splitter line -------------- " << std::endl;
}

ImuTracker::ImuTracker()
    : imu_gravity_time_constant_(10.0),
      gravity_constant_(9.806),
      processed_time_(common::Time::min()),
      curr_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(-Eigen::Vector3d::UnitZ()),
      last_angular_velocity_(Eigen::Vector3d::Zero()),
      last_linear_accel_corrected_(Eigen::Vector3d::Zero())
{
    std::cout << "ImuTracker initial position: " << position_.x() 
                << " " << position_.y() << " " << position_.z() << std::endl;
    std::cout << " -------------- splitter line -------------- " << std::endl;
}

ImuTracker::~ImuTracker(){}

void ImuTracker::addImu(ImuData imu_data)
{
    if (!validateData(imu_data)) {
        cout << "Invalid imu data!" << endl;
        return;
    } num_imu_data_++;
    
    /* 如果是第一帧数据 */
    if (gravity_queue_.empty()){
        imu_queue_.push(imu_data);
        gravity_vector_ = imu_data.linear_acceleration;
        const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
            -gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
        orientation_ = (orientation_ * rotation).normalized();
        gravity_queue_.push(TimedGravity(imu_data.time, gravity_vector_));
        ort_queue_.push(TimedPose(imu_data.time, orientation_));
        curr_time_ = imu_data.time;
        processed_time_ = imu_data.time;
        last_linear_accel_corrected_ = imu_data.linear_acceleration;
        last_angular_velocity_ = imu_data.angular_velocity;
        return;
    }

    if (imu_data.time<=processed_time_) {
        cout << "WARNING: Input imu date older than processed data." << endl;
        return;
    }

    /* 正常处理流程 */
    {
        curr_time_ = imu_data.time;
        predictOrientation(imu_data.angular_velocity);
        correctOrientationWithGravity(imu_data.linear_acceleration);
        processed_time_ = curr_time_;
        last_linear_accel_corrected_ = imu_data.linear_acceleration;
        last_angular_velocity_ = imu_data.angular_velocity;
        trimQueue();
        return;
    }
}



void ImuTracker::predictOrientation(const Eigen::Vector3d& angular_velocity) {
    // step#1
    const double delta_t = common::ToSeconds(curr_time_ - processed_time_);
    if(delta_t > 0.5){
        // 输出警告：时间差大于0.5s，可能造成误差过大。
    }
    // step#2
    const Eigen::Quaterniond rotation = AngleAxisVectorToRotationQuaternion(
            Eigen::Vector3d((angular_velocity+last_angular_velocity_)/0.5*delta_t));
    // step#3
    orientation_ = (orientation_ * rotation).normalized();
    // step#4
    gravity_vector_ = rotation.conjugate() * gravity_vector_;
    return;
}



void ImuTracker::correctOrientationWithGravity(const Eigen::Vector3d& linear_accel) {
    // step#1 
    const double delta_t = common::ToSeconds(curr_time_ - processed_time_);

    // step#2 对新时刻的重力方向测量进行滤波，起到平滑作用
    const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
    gravity_vector_ =
        (1. - alpha) * gravity_vector_ + alpha * linear_accel;
    // step#3 根据重力方向，矫正orientation_
    const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
        -gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
    orientation_ = (orientation_ * rotation).normalized();

    // step#4 TODO: 对线加速度“伪积分”，计算位移
    // position_.x() = position_.x() + linear_accel.x() * delta_t;
    // position_.y() = position_.y() + linear_accel.y() * delta_t;

    return;
}



Eigen::Quaterniond ImuTracker::AngleAxisVectorToRotationQuaternion(
        const Eigen::Matrix<double, 3, 1>& angle_axis) 
{
    double scale = double(0.5);
    double w = double(1.);
    constexpr double kCutoffAngle = 1e-8;  // We linearize below this angle.
    if (angle_axis.squaredNorm() > kCutoffAngle) {
        const double norm = angle_axis.norm();
        scale = sin(norm / 2.) / norm;
        w = cos(norm / 2.);
    }
    const Eigen::Matrix<double, 3, 1> quaternion_xyz = scale * angle_axis;
    return Eigen::Quaternion<double>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                                quaternion_xyz.z());
}


void ImuTracker::trimQueue()
{
    if (common::ToSeconds(common::Duration(
            imu_queue_.back().time-imu_queue_.front().time))>q_duration_)
    imu_queue_.pop();
    if (common::ToSeconds(common::Duration(
            ort_queue_.back().time-ort_queue_.front().time))>q_duration_)
    ort_queue_.pop();
    if (common::ToSeconds(common::Duration(
            gravity_queue_.back().time-gravity_queue_.front().time))>q_duration_)
    gravity_queue_.pop();
}


// ############################################################################
// Eigen::Quaterniond ImuTracker::FromTwoVectors(const Eigen::Vector3d& a,
//                                   const Eigen::Vector3d& b) {
//   return Eigen::Quaterniond::FromTwoVectors(a, b);
// }
// euler_angle_ = orientation_.normalized().toRotationMatrix().eulerAngles(2,1,0)/M_PI*180.0;
// LOG(WARNING)<<"wgh--#### orientation_ after  gravity alignment: "
//   << euler_angle_[0] << " " << euler_angle_[1] << " " << euler_angle_[2];
// LOG(WARNING)<< "";
// ############################################################################