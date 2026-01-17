// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Implementation of Transform_ member functions.
// Do not include directly; included by transform.hpp.

#ifndef NANOPCL_TRANSFORM_IMPL_TRANSFORM_IMPL_HPP
#define NANOPCL_TRANSFORM_IMPL_TRANSFORM_IMPL_HPP

#include "nanopcl/transform/math.hpp"

namespace npcl {

// =============================================================================
// Factory Methods
// =============================================================================

template <typename Scalar>
Transform_<Scalar> Transform_<Scalar>::fromRPY(const std::string& parent,
                                               const std::string& child,
                                               Scalar roll, Scalar pitch,
                                               Scalar yaw,
                                               const Vector3& translation) {
  Isometry T = math::isometryFromRPY(roll, pitch, yaw, translation);
  return Transform_(parent, child, T);
}

template <typename Scalar>
Transform_<Scalar> Transform_<Scalar>::from2D(const std::string& parent,
                                              const std::string& child,
                                              Scalar x, Scalar y, Scalar yaw) {
  Isometry T = math::isometryFrom2D(x, y, yaw);
  return Transform_(parent, child, T);
}

// =============================================================================
// Accessors
// =============================================================================

template <typename Scalar>
Scalar Transform_<Scalar>::roll() const {
  return math::getRoll(T_.rotation());
}

template <typename Scalar>
Scalar Transform_<Scalar>::pitch() const {
  return math::getPitch(T_.rotation());
}

template <typename Scalar>
Scalar Transform_<Scalar>::yaw() const {
  return math::getYaw(T_.rotation());
}

template <typename Scalar>
void Transform_<Scalar>::getRPY(Scalar& roll, Scalar& pitch,
                                Scalar& yaw_out) const {
  math::getRPY(T_.rotation(), roll, pitch, yaw_out);
}

// =============================================================================
// Interpolation
// =============================================================================

template <typename Scalar>
Transform_<Scalar> Transform_<Scalar>::slerp(const Transform_& other,
                                             Scalar t) const {
  if (child_ != other.child_ || parent_ != other.parent_) {
    throw std::runtime_error("slerp requires matching frames");
  }
  Isometry interp = math::slerp(T_, other.T_, t);
  Transform_ result(parent_, child_, interp);

  // Linearly interpolate timestamp
  auto dt = static_cast<Scalar>(other.stamp_) - static_cast<Scalar>(stamp_);
  result.setTimestamp(stamp_ + static_cast<Timestamp>(dt * t));
  return result;
}

}  // namespace npcl

#endif  // NANOPCL_TRANSFORM_IMPL_TRANSFORM_IMPL_HPP
