// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for SO3.
// Do not include this file directly; include <nanopcl/transform/so3.hpp>

#ifndef NANOPCL_TRANSFORM_IMPL_SO3_IMPL_HPP
#define NANOPCL_TRANSFORM_IMPL_SO3_IMPL_HPP

#include <algorithm>
#include <cmath>
#include <string>

namespace nanopcl {

// ========== Factory Methods ==========

template <typename Scalar>
SO3_<Scalar> SO3_<Scalar>::fromRPY(Scalar roll, Scalar pitch, Scalar yaw) {
  return SO3_(Quaternion(AngleAxis(yaw, Vector3::UnitZ()) *
                         AngleAxis(pitch, Vector3::UnitY()) *
                         AngleAxis(roll, Vector3::UnitX())));
}

template <typename Scalar>
SO3_<Scalar> SO3_<Scalar>::fromAngleAxis(Scalar angle, const Vector3& axis) {
  return SO3_(Quaternion(AngleAxis(angle, axis.normalized())));
}

// ========== Lie Algebra Operations ==========

template <typename Scalar>
SO3_<Scalar> SO3_<Scalar>::exp(const Vector3& omega) {
  Scalar angle = omega.norm();
  if (angle < Scalar(1e-10)) {
    return SO3_::Identity();
  }
  Vector3 axis = omega / angle;
  return SO3_(Quaternion(AngleAxis(angle, axis)));
}

template <typename Scalar>
typename SO3_<Scalar>::Vector3 SO3_<Scalar>::log() const {
  AngleAxis aa(rotation_);
  Scalar angle = aa.angle();
  if (angle < Scalar(1e-10)) {
    return Vector3::Zero();
  }
  return angle * aa.axis();
}

// ========== Accessors ==========

template <typename Scalar>
void SO3_<Scalar>::getRPY(Scalar& roll, Scalar& pitch, Scalar& yaw) const {
  auto m = matrix();
  pitch = std::asin(std::clamp(-m(2, 0), Scalar(-1), Scalar(1)));

  if (std::abs(std::cos(pitch)) > Scalar(1e-6)) {
    roll = std::atan2(m(2, 1), m(2, 2));
    yaw = std::atan2(m(1, 0), m(0, 0));
  } else {
    roll = Scalar(0);
    yaw = std::atan2(-m(0, 1), m(1, 1));
  }
}

// ========== Utilities ==========

template <typename Scalar>
std::string SO3_<Scalar>::toString() const {
  Scalar roll, pitch, yaw;
  getRPY(roll, pitch, yaw);
  return "SO3(RPY: [" + std::to_string(roll) + ", " + std::to_string(pitch) +
         ", " + std::to_string(yaw) + "])";
}

}  // namespace nanopcl

#endif  // NANOPCL_TRANSFORM_IMPL_SO3_IMPL_HPP
