/**
 * @warning: 所使用的imu必须是右手系类型！！
 * @author: wgh--
 * @description: 追踪imu坐标系的姿态角，通过角速度积分做预测，再用重力方向做矫正。
 * @dependencies: C++, Eigen, C++11::std::chrono, C++11::std::ratio
 *      类模板 std::ratio 及相关的模板提供编译时有理数算术支持
 * @caller: called by who? --class PoseTracker, or whoever needs to track imu_frame pose
 */

#ifndef CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
#define CARTOGRAPHER_MAPPING_IMU_TRACKER_H_

// C++
#include<vector>
#include<queue>
#include<string>
// Eigen
//
// 自定义
#include"liv_time.h"
#include"liv_utils.h"

using namespace std;


// wgh-- 解释：假设机器人缓慢移动，则整体来看，加速度测量的均值就是重力加速度。
// wgh-- 解释：重力加速度可以用于矫正roll和pitch的漂移！
// wgh-- 重点：仅用于估算orientation！
// Keeps track of the orientation using angular velocities and linear
// accelerations from an IMU. Because averaged linear acceleration (assuming
// slow movement) is a direct measurement of gravity, roll/pitch does not drift,
// though yaw does.
class ImuTracker {

  public:
    explicit ImuTracker(double imu_gravity_time_constant);
    ImuTracker();
    ~ImuTracker();

    // Interfaces
    void addImu(ImuData imu_data);

    Eigen::Matrix3d getGravity();
    Eigen::Matrix3d getDeltaOrientation();

    // Query the current time, and the current estimate of orientation and position.
    common::Time time() const { return processed_time_; }
    Eigen::Quaterniond orientation() const { return orientation_; }
    Eigen::Vector3d position() const { return position_; }


  private:
    // wgh-- 用imu的角速度值做积分，预测新时刻的姿态(世界坐标系)以及重力方向(imu坐标系)
    // wgh-- 用imu的重力方向观测(线加速度)，来矫正姿态预测
    inline void predictOrientation(const Eigen::Vector3d& angular_velocity);
    inline void correctOrientationWithGravity(const Eigen::Vector3d& linear_accel);

    // Returns a quaternion representing the same rotation as the given 'angle_axis' vector.
    inline Eigen::Quaterniond AngleAxisVectorToRotationQuaternion(const Eigen::Matrix<double, 3, 1>& angle_axis);

    inline void trimQueue();

    inline bool validateData(const ImuData& input_data){
        if (input_data.time==common::Time::min()
            || input_data.linear_acceleration==Eigen::Vector3d::Zero()
            || input_data.angular_velocity==Eigen::Vector3d::Zero()) {
                return false;
        } return true;
    }


    // containers
    std::queue<ImuData> imu_queue_;              // wgh-- queue for unprocessed imu data
    std::queue<TimedPose> ort_queue_;
    std::queue<TimedGravity> gravity_queue_;
    double q_duration_ = 5.0;
    // static parameters
    const double imu_gravity_time_constant_;      // wgh-- 一阶低通滤波参数，平滑更新gravity_vector_
    const double gravity_constant_;
    // calculation variables
    Eigen::Quaterniond orientation_;              // wgh-- 核心变量！
    Eigen::Vector3d gravity_vector_;              // wgh-- 核心变量！
    Eigen::Vector3d last_angular_velocity_; 
    Eigen::Vector3d last_linear_accel_corrected_;
    // run-time variables
    size_t num_imu_data_ = 0;                          // wgh-- imu data帧计数
    common::Time processed_time_;                 // wgh-- 当前最新已处理数据的时间
    common::Time curr_time_;                      // wgh-- 实时传入的数据的时间戳
    // 加速度->速度->位置积分（中值积分法）
    bool extrapolate_position_ = false;
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();                    // wgh-- 根据传感器数据实时更新
};


#endif  // CARTOGRAPHER_MAPPING_IMU_TRACKER_H_
