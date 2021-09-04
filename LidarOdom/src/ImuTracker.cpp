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
    : IMU_GRAVITY_TIME_CONST(imu_gravity_time_constant),
      GRAVITY_CONST(9.806),
      processed_time_(common::Time::min()),
      curr_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(-Eigen::Vector3d::UnitZ()*9.8),
      last_angular_velocity_(Eigen::Vector3d::Zero()),
      last_linear_accel_corrected_(Eigen::Vector3d::Zero())
{
    std::cout << "ImuTracker initial position: " << position_.x() 
                << " " << position_.y() << " " << position_.z() << std::endl;
    std::cout << " -------------- splitter line -------------- " << std::endl;
}

ImuTracker::ImuTracker()
    : IMU_GRAVITY_TIME_CONST(5.0),
      GRAVITY_CONST(9.806),
      processed_time_(common::Time::min()),
      curr_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(-Eigen::Vector3d::UnitZ()*9.8),
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
    } 
    num_received_imu++;
    
    /* 如果是第一帧数据 */
    if (num_received_imu==1){
        imu_queue_.push_back(imu_data);
        gravity_vector_ = imu_data.linear_acceleration;
        gravity_queue_.push_back(TimedGravity(imu_data.time, gravity_vector_));
        const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
            -gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
        orientation_ = (orientation_ * rotation).normalized();
        ort_queue_.push_back(TimedOrient(imu_data.time, orientation_));
        orient_WithoutG_ = orientation_;
        ort_WithoutG_queue_.push_back(TimedOrient(imu_data.time, orient_WithoutG_));
        last_angular_velocity_ = imu_data.angular_velocity;
        last_linear_accel_corrected_ = imu_data.linear_acceleration;
        curr_time_ = imu_data.time;
        processed_time_ = imu_data.time;
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

        imu_queue_.push_back(imu_data);
        ort_WithoutG_queue_.push_back(TimedOrient(curr_time_, orient_WithoutG_));
        gravity_queue_.push_back(TimedGravity(curr_time_,gravity_vector_));
        ort_queue_.push_back(TimedOrient(imu_data.time, orientation_));
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
            Eigen::Vector3d((angular_velocity+last_angular_velocity_)/2.0*delta_t));
    // step#3
    orientation_ = (orientation_ * rotation).normalized();
    orient_WithoutG_ = (orient_WithoutG_ * rotation).normalized();
    // step#4
    gravity_vector_ = rotation.conjugate() * gravity_vector_;
    return;
}



void ImuTracker::correctOrientationWithGravity(const Eigen::Vector3d& linear_accel)
{
    // step#1 
    const double delta_t = common::ToSeconds(curr_time_ - processed_time_);

    // step#2 对新时刻的重力方向测量进行滤波，起到平滑作用
    const double alpha = 1. - std::exp(-delta_t / IMU_GRAVITY_TIME_CONST);
    gravity_vector_ =
        (1. - alpha) * gravity_vector_ + alpha * linear_accel;

    // step#3 根据重力方向，矫正orientation_
    const Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(
        -gravity_vector_, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
    orientation_ = (orientation_ * rotation).normalized();

    return;
}



Eigen::Vector3d ImuTracker::getGravity(common::Time t_query)
{
    // You cannot query a deque when it's empty!
    if (gravity_queue_.empty()) return gravity_vector_;

    // check time.
    if (t_query<gravity_queue_.front().time || 
        t_query>gravity_queue_.back().time + common::FromSeconds(1.0) ) {
        std::cout << "ERROR! Time point exceeds valid scope!" << std::endl;
        return -Eigen::Vector3d::UnitZ()*GRAVITY_CONST;
    }

    // search from the queue end to queue start.
    // Usually the query time point is near the end, by doing so we save time.
    // Since all msgs are dispatched in a strict chronological order, maybe
    // we dont have to extrapolate.
    return gravity_vector_;
}


Eigen::Quaterniond ImuTracker::getAlignedOrientation(common::Time t_query)
{
    return orientation_; 
}


Eigen::Quaterniond ImuTracker::getDeltaOrientationSinceTime(common::Time t_start)
{
    // check time.
    if ( t_start<=ort_queue_.front().time ) {
        std::cout << "ERROR! Start time point exceeds valid scope!" << std::endl;
        return Eigen::Quaterniond::Identity();
    }
    Eigen::Quaterniond endTimeOrient = ort_queue_.back().orient;
    Eigen::Quaterniond startTimeOrient;
    for (std::size_t i=ort_queue_.size()-1; i>=0; i--) {
        if (ort_queue_[i].time<t_start) {
            startTimeOrient = ort_queue_[i].orient;
            break;
        }
    }
    return (startTimeOrient.conjugate() * endTimeOrient).normalized(); 
}


Eigen::Quaterniond ImuTracker::getDeltaOrientation(common::Time t_start, common::Time t_end)
{
    // check time.
    if (!(t_start<=t_end)) {
        std::cout << "ERROR! Start time must be earlier than the End!" << std::endl;
        return Eigen::Quaterniond::Identity();
    }
    if ( t_start<=ort_queue_.front().time ) {
        std::cout << "ERROR! Start time point exceeds valid scope!" << std::endl;
        return Eigen::Quaterniond::Identity();
    }
    // we allow the t_end to be later than the last orientation in queue.
    if ( t_end>(ort_queue_.back().time+common::FromMilliseconds(10)) ) {
        std::cout << "WARNING! End time point is too new than ImuTracker." << std::endl;
    }

    bool endTimeFound = false, startTimeFound = false;
    Eigen::Quaterniond endTimeOrient, startTimeOrient;
    for (std::size_t i=ort_queue_.size()-1; i>=0; i--) {
        if (!endTimeFound) {
            if (ort_queue_[i].time<t_end) {
                // if (i==ort_queue_.size()-1) { endTimeOrient = ort_queue_[i].orient; }
                // else {
                //     double ratio = (t_end-ort_queue_[i].time)*1.0/
                //                     (ort_queue_[i+1].time-ort_queue_[i].time) ;
                //     // endTimeOrient = ratio*ort_queue_[i].orient 
                //     //                 + (1-ratio)*ort_queue_[i+1].orient;
                // }
                endTimeOrient = ort_queue_[i].orient;
                endTimeFound = true;
            }
        }
        if (!startTimeFound) {
            if (ort_queue_[i].time<t_start) {
                startTimeOrient = ort_queue_[i].orient;
                startTimeFound = true;
                break;
            }
        }
    }

    return (startTimeOrient.conjugate() * endTimeOrient).normalized(); 
}


// 暂不启用
Eigen::Quaterniond ImuTracker::getDeltaOrientationPureRot
                    (common::Time t_start, common::Time t_end)
{
    // check time.
    if (!(t_start<t_end)) {
        std::cout << "ERROR! Start time must be earlier than the End!" << std::endl;
        return Eigen::Quaterniond::Identity();
    }
    if (t_start<ort_WithoutG_queue_.front().time 
        || t_end>ort_WithoutG_queue_.back().time) 
    {
        std::cout << "ERROR! Time point exceeds valid scope!" << std::endl;
        return Eigen::Quaterniond::Identity();
    }

    return Eigen::Quaterniond::Identity();
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
            imu_queue_.back().time-imu_queue_.front().time))>QUEUE_DURATION)
    imu_queue_.pop_front();
    if (common::ToSeconds(common::Duration(
            gravity_queue_.back().time-gravity_queue_.front().time))>QUEUE_DURATION)
    gravity_queue_.pop_front();
    if (common::ToSeconds(common::Duration(
            ort_queue_.back().time-ort_queue_.front().time))>QUEUE_DURATION)
    ort_queue_.pop_front();
    if (common::ToSeconds(common::Duration(
            ort_WithoutG_queue_.back().time-ort_WithoutG_queue_.front().time))
        > QUEUE_DURATION)
    ort_WithoutG_queue_.pop_front();
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