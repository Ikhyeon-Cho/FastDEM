// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for SE3.
// Do not include this file directly; include <nanopcl/transform/se3.hpp>

#ifndef NANOPCL_TRANSFORM_IMPL_SE3_IMPL_HPP
#define NANOPCL_TRANSFORM_IMPL_SE3_IMPL_HPP

#include <algorithm>
#include <cmath>
#include <string>

namespace nanopcl {

// ========== Private Helper Functions ==========

template <typename Scalar>
typename SE3_<Scalar>::Matrix3 SE3_<Scalar>::skew(const Vector3& v) {
  Matrix3 s;
  s << Scalar(0), -v.z(), v.y(),
       v.z(), Scalar(0), -v.x(),
       -v.y(), v.x(), Scalar(0);
  return s;
}

template <typename Scalar>
typename SE3_<Scalar>::Matrix3 SE3_<Scalar>::leftJacobian(const Vector3& omega) {
  Scalar theta = omega.norm();
  if (theta < Scalar(1e-10)) {
    return Matrix3::Identity();
  }
  Vector3 axis = omega / theta;
  Matrix3 omega_hat = skew(axis);
  Scalar c1 = (Scalar(1) - std::cos(theta)) / (theta * theta);
  Scalar c2 = (theta - std::sin(theta)) / (theta * theta * theta);
  return Matrix3::Identity() + c1 * omega_hat + c2 * omega_hat * omega_hat;
}

template <typename Scalar>
typename SE3_<Scalar>::Matrix3 SE3_<Scalar>::leftJacobianInverse(
    const Vector3& omega) {
  Scalar theta = omega.norm();
  if (theta < Scalar(1e-10)) {
    return Matrix3::Identity();
  }
  Vector3 axis = omega / theta;
  Matrix3 omega_hat = skew(axis);
  Scalar half_theta = Scalar(0.5) * theta;
  Scalar c = Scalar(1) / (theta * theta) -
             (Scalar(1) + std::cos(theta)) / (Scalar(2) * theta * std::sin(theta));
  return Matrix3::Identity() - half_theta * omega_hat + c * omega_hat * omega_hat;
}

// ========== Factory Methods ==========

template <typename Scalar>
SE3_<Scalar> SE3_<Scalar>::from2D(Scalar x, Scalar y, Scalar theta) {
  SE3_ tf;
  tf.transform_.translation() = Vector3(x, y, 0);
  tf.transform_.rotate(Eigen::AngleAxis<Scalar>(theta, Vector3::UnitZ()));
  return tf;
}

template <typename Scalar>
SE3_<Scalar> SE3_<Scalar>::fromRPY(Scalar roll, Scalar pitch, Scalar yaw) {
  Eigen::AngleAxis<Scalar> rollAngle(roll, Vector3::UnitX());
  Eigen::AngleAxis<Scalar> pitchAngle(pitch, Vector3::UnitY());
  Eigen::AngleAxis<Scalar> yawAngle(yaw, Vector3::UnitZ());

  Quaternion q = yawAngle * pitchAngle * rollAngle;
  SE3_ tf;
  tf.transform_.rotate(q);
  return tf;
}

template <typename Scalar>
SE3_<Scalar> SE3_<Scalar>::fromQuaternion(Scalar qx, Scalar qy, Scalar qz,
                                           Scalar qw, const Vector3& translation) {
  Quaternion q(qw, qx, qy, qz);
  return SE3_(translation, q);
}

template <typename Scalar>
SE3_<Scalar> SE3_<Scalar>::fromQuaternion(const Quaternion& q,
                                           const Vector3& translation) {
  return SE3_(translation, q);
}

template <typename Scalar>
SE3_<Scalar> SE3_<Scalar>::fromRotation(const SO3_<Scalar>& rotation,
                                         const Vector3& translation) {
  return SE3_(translation, rotation.matrix());
}

// ========== Lie Algebra Operations ==========

template <typename Scalar>
SE3_<Scalar> SE3_<Scalar>::exp(const Vector6& xi) {
  Vector3 v = xi.template head<3>();
  Vector3 omega = xi.template tail<3>();
  SO3_<Scalar> R = SO3_<Scalar>::exp(omega);
  Matrix3 V = leftJacobian(omega);
  Vector3 t = V * v;
  return SE3_(t, R);
}

template <typename Scalar>
typename SE3_<Scalar>::Vector6 SE3_<Scalar>::log() const {
  SO3_<Scalar> R = rotation();
  Vector3 omega = R.log();
  Matrix3 V_inv = leftJacobianInverse(omega);
  Vector3 v = V_inv * translation();
  Vector6 xi;
  xi << v, omega;
  return xi;
}

// ========== Accessors ==========

template <typename Scalar>
void SE3_<Scalar>::getRPY(Scalar& roll, Scalar& pitch, Scalar& yaw) const {
  auto m = transform_.rotation();
  pitch = std::asin(std::clamp(-m(2, 0), Scalar(-1), Scalar(1)));
  if (std::abs(std::cos(pitch)) > Scalar(1e-6)) {
    roll = std::atan2(m(2, 1), m(2, 2));
    yaw = std::atan2(m(1, 0), m(0, 0));
  } else {
    roll = Scalar(0);
    yaw = std::atan2(-m(0, 1), m(1, 1));
  }
}

// ========== Interpolation ==========

template <typename Scalar>
SE3_<Scalar> SE3_<Scalar>::slerp(const SE3_& other, Scalar t) const {
  Vector3 trans = (Scalar(1) - t) * translation() + t * other.translation();
  SO3_<Scalar> rot = rotation().slerp(other.rotation(), t);
  return SE3_(trans, rot);
}

// ========== Utilities ==========

template <typename Scalar>
std::string SE3_<Scalar>::toString() const {
  Scalar roll, pitch, yaw;
  getRPY(roll, pitch, yaw);
  return "SE3(t: [" + std::to_string(x()) + ", " + std::to_string(y()) + ", " +
         std::to_string(z()) + "], RPY: [" + std::to_string(roll) + ", " +
         std::to_string(pitch) + ", " + std::to_string(yaw) + "])";
}

}  // namespace nanopcl

#endif  // NANOPCL_TRANSFORM_IMPL_SE3_IMPL_HPP
