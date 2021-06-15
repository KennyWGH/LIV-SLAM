/**
 * 
 * 
 */


#include "ImuTracker.h"
#include "time.h"

#include <cmath>
#include <limits>
#include <iostream>




ImuTracker::ImuTracker(const double imu_gravity_time_constant)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      gravity_constant_(9.806),
      num_imu_data_(0),
      processed_time_(common::Time::min()),
      data_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(-Eigen::Vector3d::UnitZ()),
      curr_angular_velocity_(Eigen::Vector3d::Zero()),
      last_angular_velocity_(Eigen::Vector3d::Zero()),
      position_(Eigen::Vector3d::Zero()),
      last_linear_accel_corrected_(Eigen::Vector3d::Zero()),
      curr_linear_accel_corrected_(Eigen::Vector3d::Zero())
{
  std::cout << "ImuTracker initial position: " << position_.x() 
            << " " << position_.y() << " " << position_.z() << std::endl;
  std::cout << " -------------- splitter line -------------- " << std::endl;
}

ImuTracker::ImuTracker()
    : imu_gravity_time_constant_(10.0),
      gravity_constant_(9.806),
      num_imu_data_(0),
      processed_time_(common::Time::min()),
      data_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(-Eigen::Vector3d::UnitZ()),
      curr_angular_velocity_(Eigen::Vector3d::Zero()),
      last_angular_velocity_(Eigen::Vector3d::Zero()),
      position_(Eigen::Vector3d::Zero()),
      last_linear_accel_corrected_(Eigen::Vector3d::Zero()),
      curr_linear_accel_corrected_(Eigen::Vector3d::Zero())
{
  std::cout << "ImuTracker initial position: " << position_.x() 
            << " " << position_.y() << " " << position_.z() << std::endl;
  std::cout << " -------------- splitter line -------------- " << std::endl;
}

ImuTracker::~ImuTracker(){}



void ImuTracker::processWithGravity(const cartographer::ImuData& imu_data){
  // step#1
  if(imu_data.time < processed_time_){
    // 输出警告：时间戳回退，无效！
    return;
  }
  // step#2
  num_imu_data_++;
  data_time_ = imu_data.time;
  predictOrientation(imu_data.angular_velocity);
  correctOrientationWithGravity(imu_data.linear_acceleration);
  processed_time_ = data_time_;
  return;
}



bool ImuTracker::predictOrientation(const Eigen::Vector3d& imu_angular_velocity) {
  // step#1 
  const double delta_t =
      processed_time_ > common::Time::min()
          ? common::ToSeconds(data_time_ - processed_time_)
          : 0.0;
  if(delta_t > 0.5){
    // 输出警告：时间差大于0.5s，可能造成误差过大。
  }

  // consider：角速度积分是否需要中值积分？？？？ #####################################

  // step#2 角速度积分
  const Eigen::Quaterniond rotation = AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity * delta_t));
  // step#3 预测新时刻的orientation_
  orientation_ = (orientation_ * rotation).normalized();
  // step#4 更新重力方向（当前帧imu坐标系下、预测值）
  gravity_vector_ = rotation.conjugate() * gravity_vector_;
  return true;
}



bool ImuTracker::correctOrientationWithGravity(const Eigen::Vector3d& imu_linear_acceleration) {
  // step#1 
  const double delta_t =
      processed_time_ > common::Time::min()
          ? common::ToSeconds(data_time_ - processed_time_)
          : std::numeric_limits<double>::infinity();

  // step#2 对新时刻的重力方向测量进行滤波，起到平滑作用
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  gravity_vector_ =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;
  // step#3 根据新的重力方向，矫正orientation_
  const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
      -gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  orientation_ = (orientation_ * rotation).normalized();

  // step#4 TODO: 对线加速度“伪积分”，计算位移
  // position_.x() = position_.x() + imu_linear_acceleration.x() * delta_t;
  // position_.y() = position_.y() + imu_linear_acceleration.y() * delta_t;

  // step#5 校验结果？
  // CHECK_GT((orientation_ * gravity_vector_).z(), 0.);
  // CHECK_GT((orientation_ * gravity_vector_).normalized().z(), 0.99);
  if( !(  (orientation_ * gravity_vector_).z()>0.0 
          && (orientation_ * gravity_vector_).normalized().z()>0.99 ) ){
    // 输出警告：结果可能错误。
    std::cout << "orientation_ CHECK wrong~" << std::endl ;
    return false;
  }
  return true;
}



Eigen::Quaterniond ImuTracker::AngleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<double, 3, 1>& angle_axis) {
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




// Eigen::Quaterniond ImuTracker::FromTwoVectors(const Eigen::Vector3d& a,
//                                   const Eigen::Vector3d& b) {
//   return Eigen::Quaterniond::FromTwoVectors(a, b);
// }

  // euler_angle_ = orientation_.normalized().toRotationMatrix().eulerAngles(2,1,0)/M_PI*180.0;
  // LOG(WARNING)<<"wgh--#### orientation_ after  gravity alignment: "
  //   << euler_angle_[0] << " " << euler_angle_[1] << " " << euler_angle_[2];
  // LOG(WARNING)<< "";