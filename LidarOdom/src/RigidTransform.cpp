
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"

#include "RigidTransform.h"

Rigid3d operator*(const Rigid3d& lhs, const Rigid3d& rhs) 
{
  return Rigid3d(
      lhs.rotation() * rhs.translation() + lhs.translation(),
      (lhs.rotation() * rhs.rotation()).normalized());
}

Rigid3d::Vector operator*(
    const Rigid3d& rigid,
    const Rigid3d::Vector& point) {
  return rigid.rotation() * point + rigid.translation();
}

Eigen::Quaterniond RollPitchYaw(const double roll, const double pitch,
                                const double yaw) 
{
  const Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
  return yaw_angle * pitch_angle * roll_angle;
}

double getAngle(const Rigid3d& transform)
{
    return 2.0 * std::atan2(transform.rotation().vec().norm(),
                            std::abs(transform.rotation().w()));
}



