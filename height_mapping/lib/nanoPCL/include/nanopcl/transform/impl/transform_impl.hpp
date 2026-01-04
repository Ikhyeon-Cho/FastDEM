// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for Transform.
// Do not include this file directly; include <nanopcl/transform/transform.hpp>

#ifndef NANOPCL_TRANSFORM_IMPL_TRANSFORM_IMPL_HPP
#define NANOPCL_TRANSFORM_IMPL_TRANSFORM_IMPL_HPP

namespace nanopcl {

// ========== Operations ==========

template <typename Scalar>
Transform_<Scalar> Transform_<Scalar>::operator*(const Transform_& other) const {
  if (!isValid()) {
    throw std::runtime_error("Transform composition: left operand not initialized");
  }
  if (!other.isValid()) {
    throw std::runtime_error("Transform composition: right operand not initialized");
  }
  if (source_ != other.target_) {
    throw std::runtime_error("Transform composition frame mismatch: '" +
                             source_ + "' != '" + other.target_ + "'");
  }
  return Transform_(T_ * other.T_, other.source_, target_);
}

template <typename Scalar>
Transform_<Scalar>& Transform_<Scalar>::operator*=(const Transform_& other) {
  if (!isValid()) {
    throw std::runtime_error("Transform composition: left operand not initialized");
  }
  if (!other.isValid()) {
    throw std::runtime_error("Transform composition: right operand not initialized");
  }
  if (source_ != other.target_) {
    throw std::runtime_error("Transform composition frame mismatch: '" +
                             source_ + "' != '" + other.target_ + "'");
  }
  T_ = T_ * other.T_;
  source_ = other.source_;
  return *this;
}

// ========== Interpolation ==========

template <typename Scalar>
Transform_<Scalar> Transform_<Scalar>::slerp(const Transform_& other,
                                              Scalar t) const {
  if (source_ != other.source_ || target_ != other.target_) {
    throw std::runtime_error("slerp requires matching frames");
  }
  return Transform_(T_.slerp(other.T_, t), source_, target_);
}

// ========== Utilities ==========

template <typename Scalar>
std::string Transform_<Scalar>::toString() const {
  return "Transform [" + source_ + " -> " + target_ + "]";
}

}  // namespace nanopcl

#endif  // NANOPCL_TRANSFORM_IMPL_TRANSFORM_IMPL_HPP
