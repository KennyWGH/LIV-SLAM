
#ifndef RIGID_TRANSFORM_H_
#define RIGID_TRANSFORM_H_

#include <cmath>
#include <iostream>
#include <string>

#include "Eigen/Core"
#include "Eigen/Geometry"


class Rigid3d {
 public:
  using Vector = Eigen::Matrix<double, 3, 1>;
  using Quaternion = Eigen::Quaternion<double>;
  using AngleAxis = Eigen::AngleAxis<double>;

  Rigid3d() : translation_(Vector::Zero()), rotation_(Quaternion::Identity()) {}
  Rigid3d(const Vector& translation, const Quaternion& rotation)
      : translation_(translation), rotation_(rotation) {}
  Rigid3d(const Vector& translation, const AngleAxis& rotation)
      : translation_(translation), rotation_(rotation) {}

  static Rigid3d Rotation(const AngleAxis& angle_axis) {
    return Rigid3d(Vector::Zero(), Quaternion(angle_axis));
  }

  static Rigid3d Rotation(const Quaternion& rotation) {
    return Rigid3d(Vector::Zero(), rotation);
  }

  static Rigid3d Translation(const Vector& vector) {
    return Rigid3d(vector, Quaternion::Identity());
  }

  static Rigid3d FromArrays(const std::array<double, 4>& rotation,
                           const std::array<double, 3>& translation) {
    return Rigid3d(Eigen::Map<const Vector>(translation.data()),
                  Eigen::Quaternion<double>(rotation[0], rotation[1],
                                               rotation[2], rotation[3]));
  }

  static Rigid3d Identity() { return Rigid3d(); }

  // 以上为构造相关
  // 以下为访问相关

  const Vector& translation() const { return translation_; }
  const Quaternion& rotation() const { return rotation_; }

  Rigid3d inverse() const {
    const Quaternion rotation = rotation_.conjugate();
    const Vector translation = -(rotation * translation_);
    return Rigid3d(translation, rotation);
  }

  bool IsValid() const {
    return !std::isnan(translation_.x()) && !std::isnan(translation_.y()) &&
           !std::isnan(translation_.z()) &&
           std::abs(double(1) - rotation_.norm()) < double(1e-3);
  }

 private:
  Vector translation_;
  Quaternion rotation_;
};


// 声明运算符重载
Rigid3d operator*(const Rigid3d& lhs, const Rigid3d& rhs);
Rigid3d::Vector operator*(const Rigid3d& rigid, const Rigid3d::Vector& point);


// Converts (roll, pitch, yaw) to a unit length quaternion. Based on the URDF
// specification http://wiki.ros.org/urdf/XML/joint.
Eigen::Quaterniond RollPitchYaw(double roll, double pitch, double yaw);

double getAngle(const Rigid3d& transform);

inline Rigid3d FromMat4dToRigid3d(const Eigen::Matrix4d& input_mat)
{
    Eigen::Matrix3d Rot = input_mat.block<3,3>(0,0);
    Eigen::Vector3d Trans = input_mat.block<3,1>(0,3);
    return Rigid3d(Trans, Eigen::Quaterniond(Rot));
}

inline Eigen::Matrix4d FromRigid3dToMat4d(const Rigid3d& input_rigid)
{
    Eigen::Matrix4d output_mat;
    // Eigen::Matrix3d Rot = input_rigid.rotation().toRotationMatrix();
    // Eigen::Vector3d Trans = input_rigid.translation();
    output_mat.block<3,3>(0,0) = input_rigid.rotation().toRotationMatrix();
    output_mat.block<3,1>(0,3) = input_rigid.translation();
    return output_mat;
}




#endif  // RIGID_TRANSFORM_H_
