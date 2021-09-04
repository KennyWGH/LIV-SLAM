/**
 * @warning: 所使用的imu必须是右手系类型！！
 * @author: wgh--
 * @description: 追踪imu坐标系的姿态角，通过角速度积分做预测，再用重力方向做矫正。
 * @dependencies: C++, Eigen, C++11::std::chrono, C++11::std::ratio
 *      类模板 std::ratio 及相关的模板提供编译时有理数算术支持
 * @caller: called by who? --class PoseTracker, or whoever needs to track imu_frame pose
 */

#ifndef IMU_TRACKER_H_
#define IMU_TRACKER_H_

// C++
#include<deque>
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

    Eigen::Vector3d getGravity(common::Time t_query);
    Eigen::Quaterniond getAlignedOrientation(common::Time t_query);
    Eigen::Quaterniond getDeltaOrientationSinceTime(common::Time t_start);
    Eigen::Quaterniond getDeltaOrientation(common::Time t_start, common::Time t_end);
    Eigen::Quaterniond getDeltaOrientationPureRot(common::Time t_start, common::Time t_end);

    // Query the current time, and the current estimate of orientation and position.
    common::Time time() const { return processed_time_; }
    Eigen::Quaterniond orientation() const { return orientation_; }
    Eigen::Vector3d position() const { return position_; }
    Eigen::Quaterniond orientationWithoutG() const { return orient_WithoutG_; }


  private:
    // wgh-- 用imu的角速度值做积分，预测新时刻的姿态(世界坐标系)以及重力方向(imu坐标系)
    // wgh-- 用imu的重力方向观测(线加速度)，来矫正姿态预测
    void predictOrientation(const Eigen::Vector3d& angular_velocity);
    void correctOrientationWithGravity(const Eigen::Vector3d& linear_accel);

    // Returns a quaternion representing the same rotation 
    // as the given 'angle_axis' vector.
    // This is typical used for angular velocity integration.
    Eigen::Quaterniond AngleAxisVectorToRotationQuaternion(
                            const Eigen::Matrix<double, 3, 1>& angle_axis);

    void trimQueue();

    inline bool validateData(const ImuData& input_data){
        if (input_data.time==common::Time::min()
            || input_data.linear_acceleration==Eigen::Vector3d::Zero()
            || input_data.angular_velocity==Eigen::Vector3d::Zero()) 
        { return false; } 
        return true;
    }


    // static parameters
    const double IMU_GRAVITY_TIME_CONST;      // wgh-- 一阶指数滤波参数，平滑更新重力加速度；
            /* Cartographer默认该参数为10.0 */  // wgh-- 该常数越小，新观测的影响来的就越快。
    const double GRAVITY_CONST;               // wgh-- 重力常量
    double QUEUE_DURATION = 5.0;
    // containers
    std::deque<ImuData> imu_queue_;              // wgh-- queue for unprocessed imu data
    std::deque<TimedGravity> gravity_queue_;
    std::deque<TimedOrient> ort_queue_;
    std::deque<TimedOrient> ort_WithoutG_queue_;
    
    // important variables
    Eigen::Quaterniond orientation_;              // wgh-- 核心变量！
    Eigen::Quaterniond orient_WithoutG_; 
    Eigen::Vector3d gravity_vector_;              // wgh-- 核心变量！
    Eigen::Vector3d last_angular_velocity_; 
    Eigen::Vector3d last_linear_accel_corrected_;
    // run-time variables
    size_t num_received_imu = 0;                          // wgh-- imu data帧计数
    common::Time curr_time_;                      // wgh-- 实时传入的数据的时间戳
    common::Time processed_time_;                 // wgh-- 当前最新已处理数据的时间

    // 加速度->速度->位置积分（中值积分法）
    bool extrapolate_position_ = false;
    Eigen::Vector3d position_ = Eigen::Vector3d::Zero();   // wgh-- 根据传感器数据实时更新
};


#endif  // IMU_TRACKER_H_
