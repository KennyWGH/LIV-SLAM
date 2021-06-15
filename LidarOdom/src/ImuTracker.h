/**
 * @author: wgh--
 * @description: 追踪imu坐标系的姿态角，通过角速度积分做预测，再用重力方向做矫正。
 * @dependencies: C++, Eigen, C++11::std::chrono, C++11::std::ratio
 *      类模板 std::ratio 及相关的模板提供编译时有理数算术支持
 * @caller: called by who? --class PoseTracker, or whoever needs to track imu_frame pose
 */

#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

#include "Eigen/Geometry"
#include "time.h"


struct ImuData {
    common::Time time;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
};

// struct Pose {
//     //
// };


// wgh-- 解释：假设机器人缓慢移动，则整体来看，加速度测量的均值就是重力加速度。
// wgh-- 解释：重力加速度可以用于矫正roll和pitch的漂移！
// wgh-- 重点：仅用于估算orientation！
// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
class ImuTracker {
 
 public:
  explicit ImuTracker(double imu_gravity_time_constant = 10.0);
  ImuTracker();
  ~ImuTracker();

  // Interface functions
  void processWithGravity(const ImuData& imu_data);
  void process(const ImuData& imu_data);

  void advanceWithGravity(const ImuData& imu_data);
  void advance(const ImuData& imu_data);

  // Interfaces
  Eigen::Matrix3d getGravity();
  Eigen::Matrix3d getDeltaOrientation();

  // Query the current time, and the current estimate of orientation and position.
  common::Time time() const { return processed_time_; }
  Eigen::Quaterniond orientation() const { return orientation_; }
  Eigen::Vector3d position() const { return position_; }


 private:
  // wgh-- 用imu的角速度值做积分，预测新时刻的姿态角(世界坐标系)以及重力方向(imu坐标系)
  // wgh-- 用imu的加速度值得到重力方向观测，用重力方向矫正姿态角
  bool predictOrientation(const Eigen::Vector3d& imu_angular_velocity);
  bool correctOrientationWithGravity(const Eigen::Vector3d& imu_linear_acceleration);
  // Returns a quaternion representing the same rotation as the given 'angle_axis' vector.
  Eigen::Quaterniond AngleAxisVectorToRotationQuaternion(const Eigen::Matrix<double, 3, 1>& angle_axis);
//   Eigen::Quaterniond FromTwoVectors(const Eigen::Vector3d& a, const Eigen::Vector3d& b);


  // wgh-- static parameters
  const double imu_gravity_time_constant_;      // wgh-- 一阶低通滤波参数，平滑更新gravity_vector_
  const double gravity_constant_;
  // wgh-- run-time parameters
  size_t num_imu_data_;                          // wgh-- imu data帧计数
  common::Time processed_time_;                 // wgh-- 当前最新已处理数据的时间
  common::Time data_time_;                      // wgh-- 实时传入的数据的时间戳
  // calculate orientation.
  Eigen::Quaterniond orientation_;              // wgh-- 核心变量！一切为这个变量服务
  Eigen::Vector3d gravity_vector_;              // wgh-- 根据传感器数据实时更新
  Eigen::Vector3d curr_angular_velocity_;        // wgh-- 当前帧imu角速度
  Eigen::Vector3d last_angular_velocity_;        // wgh-- 当前帧imu角速度
  // 加速度->速度->位置积分
  Eigen::Vector3d position_;                    // wgh-- 根据传感器数据实时更新
  Eigen::Vector3d last_linear_accel_corrected_;
  Eigen::Vector3d curr_linear_accel_corrected_;
};


#endif  // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
