/**
 * 
 * 
 */

#ifndef LIV_UTILS_H_
#define LIV_UTILS_H_

// C++
#include<cmath>
#include<ratio>
// Eigen
#include<Eigen/Core>
#include<Eigen/Geometry>
// 自定义
#include"liv_time.h"

struct ImuData {
    common::Time time = common::Time::min();
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();

    explicit ImuData(common::Time time_, Eigen::Vector3d accel_, 
                    Eigen::Vector3d angular_)
        :time(time_), linear_acceleration(accel_), 
                    angular_velocity(angular_) {}
};

struct TimedPose {
    common::Time time = common::Time::min();
    Eigen::Quaterniond ort = Eigen::Quaterniond::Identity();

    explicit TimedPose(common::Time time_, Eigen::Quaterniond pose_)
        : time(time_), ort(pose_) {}
};

struct TimedGravity {
    common::Time time = common::Time::min();
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();

    explicit TimedGravity(common::Time time_, Eigen::Vector3d gravity_)
        : time(time_), gravity(gravity_) {}
};

class FixedRatioSampler{
  public:
    explicit FixedRatioSampler(double ratio);
    ~FixedRatioSampler();

    FixedRatioSampler(const FixedRatioSampler&) = delete;
    FixedRatioSampler& operator=(const FixedRatioSampler&) = delete;

    bool pulse();
    double currentRatio();

  private:
    double ratio_;
    std::uint64_t num_hits_ = 0;
    std::uint64_t num_pulses_ = 0;
};

#endif // LIV_UTILS_H_
