/**
 * 
 * 
 */

#ifndef LIV_UTILS_H_
#define LIV_UTILS_H_

// C++
#include<cmath>
#include<ratio>
#include<cstdint>
// Eigen
#include<Eigen/Core>
#include<Eigen/Geometry>
// PCL
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
// 自定义
#include"liv_time.h"
#include"RigidTransform.h"

typedef pcl::PointXYZI PointType;
/* I(intensity)中，整数部分表示laserID，小数部分表示相对时间。 */

struct ImuData {
    common::Time time = common::Time::min();
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();

    ImuData(){}
    explicit ImuData(common::Time time_, Eigen::Vector3d accel_, 
                    Eigen::Vector3d angular_)
        :time(time_), linear_acceleration(accel_), 
                    angular_velocity(angular_) {}
};

struct TimedOrient {
    common::Time time = common::Time::min();
    Eigen::Quaterniond orient = Eigen::Quaterniond::Identity();

    TimedOrient(){}
    explicit TimedOrient(common::Time time_, Eigen::Quaterniond pose_)
        : time(time_), orient(pose_) {}
};

struct TimedGravity {
    common::Time time = common::Time::min();
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();

    TimedGravity(){}
    explicit TimedGravity(common::Time time_, Eigen::Vector3d gravity_)
        : time(time_), gravity(gravity_) {}
};

struct TimedPose {
    common::Time time = common::Time::min();
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond orient = Eigen::Quaterniond::Identity();

    TimedPose(){}
    explicit TimedPose(common::Time time_, 
    Eigen::Vector3d position_, Eigen::Quaterniond orient_)
        : time(time_), position(position_), orient(orient_) {}
};

struct CompletePose {
    std::size_t id = 0; /* 10000 */
    common::Time time = common::Time::min();
    // Eigen::Vector3d position = Eigen::Vector3d::Zero();
    // Eigen::Quaterniond orient = Eigen::Quaterniond::Identity();
    Rigid3d pose = Rigid3d::Identity();
    Eigen::Vector3d gravity = Eigen::Vector3d::Zero();

    /** *******************************************************
     *  经测试，显示定义的默认构造仍然会按照成员定义位置的初始化值进行初始化
     *  比如，本结构体中的成员id，若设为10000，则默认构造函数按10000初始化 
     *  ******************************************************* */
    CompletePose(){} 
    explicit CompletePose(std::size_t id_,
                        common::Time time_, 
                        Rigid3d pose_,
                        Eigen::Vector3d gravity_)
        : id(id_), time(time_), pose(pose_), gravity(gravity_) {}

    void reset() {
        id = 0;
        time = common::Time::min();
        pose = Rigid3d::Identity();
        gravity = Eigen::Vector3d::Zero();
    }
};

class FixedRatioSampler{
  public:
    explicit FixedRatioSampler(double ratio);
    FixedRatioSampler();
    ~FixedRatioSampler();

    FixedRatioSampler(const FixedRatioSampler&) = delete;
    FixedRatioSampler& operator=(const FixedRatioSampler&) = delete;

    bool pulse();
    double currentRatio();
    void setRatio(double ratio);

  private:
    double ratio_=0.1;
    std::uint64_t num_hits_ = 0;
    std::uint64_t num_pulses_ = 0;
};

#endif // LIV_UTILS_H_
