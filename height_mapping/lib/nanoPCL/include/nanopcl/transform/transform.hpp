// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_TRANSFORM_TRANSFORM_HPP
#define NANOPCL_TRANSFORM_TRANSFORM_HPP

#include <cassert>
#include <stdexcept>
#include <string>

#include "nanopcl/transform/se3.hpp"

namespace nanopcl {

// ============================================================================
// Transform - Frame-Aware SE(3) Wrapper
// ============================================================================

/**
 * @brief Frame-aware transformation wrapper for SE3
 *
 * Transform represents a rigid body transformation from one coordinate frame
 * to another. Unlike SE3 which is a pure mathematical Lie group, Transform
 * carries explicit frame semantics for robotics applications.
 *
 * @tparam Scalar Floating point type (double or float)
 */
template <typename Scalar = double>
class Transform_ {
 public:
  using SE3Type = SE3_<Scalar>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Quaternion = Eigen::Quaternion<Scalar>;

 private:
  SE3Type T_;
  std::string source_;
  std::string target_;

  Transform_(const SE3Type& T, const std::string& from, const std::string& to)
      : T_(T), source_(from), target_(to) {}

 public:
  Transform_() = default;

  // ========== Validation ==========

  bool isValid() const { return !source_.empty() && !target_.empty(); }

  // ========== Fluent Setters ==========

  Transform_& from(const std::string& frame) {
    assert(source_.empty() && "source frame already set");
    source_ = frame;
    return *this;
  }

  Transform_& to(const std::string& frame) {
    assert(target_.empty() && "target frame already set");
    target_ = frame;
    return *this;
  }

  Transform_& with(const SE3Type& T) {
    T_ = T;
    return *this;
  }

  template <int Mode>
  Transform_& with(const Eigen::Transform<Scalar, 3, Mode>& tf) {
    static_assert(Mode == Eigen::Isometry || Mode == Eigen::Affine,
                  "Only Isometry or Affine transforms are allowed");
    T_ = SE3Type(Eigen::Transform<Scalar, 3, Eigen::Isometry>(tf.matrix()));
    return *this;
  }

  // ========== Factory Methods ==========

  static Transform_ identity(const std::string& frame) {
    return Transform_(SE3Type::Identity(), frame, frame);
  }

  // ========== Accessors ==========

  const SE3Type& T() const { return T_; }
  const std::string& source_frame() const { return source_; }
  const std::string& target_frame() const { return target_; }

  SO3_<Scalar> rotation() const { return T_.rotation(); }
  Quaternion quaternion() const { return T_.quaternion(); }
  Vector3 translation() const { return T_.translation(); }

  Eigen::Quaternionf quaternionf() const {
    return quaternion().template cast<float>();
  }
  Eigen::Vector3f translationf() const {
    return translation().template cast<float>();
  }

  Scalar x() const { return T_.x(); }
  Scalar y() const { return T_.y(); }
  Scalar z() const { return T_.z(); }

  void getRPY(Scalar& roll, Scalar& pitch, Scalar& yaw) const {
    T_.getRPY(roll, pitch, yaw);
  }

  Scalar yaw() const { return T_.yaw(); }

  // ========== Operations ==========

  Transform_ inverse() const {
    assert(isValid() && "Cannot invert uninitialized transform");
    return Transform_(T_.inverse(), target_, source_);
  }

  Transform_ operator*(const Transform_& other) const {
    assert(isValid() && "Left operand not initialized");
    assert(other.isValid() && "Right operand not initialized");
    if (source_ != other.target_) {
      throw std::runtime_error("Transform composition frame mismatch: '" +
                               source_ + "' != '" + other.target_ + "'");
    }
    return Transform_(T_ * other.T_, other.source_, target_);
  }

  Transform_& operator*=(const Transform_& other) {
    assert(isValid() && "Left operand not initialized");
    assert(other.isValid() && "Right operand not initialized");
    if (source_ != other.target_) {
      throw std::runtime_error("Transform composition frame mismatch: '" +
                               source_ + "' != '" + other.target_ + "'");
    }
    T_ = T_ * other.T_;
    source_ = other.source_;
    return *this;
  }

  template <typename T>
  Eigen::Matrix<T, 3, 1> operator*(const Eigen::Matrix<T, 3, 1>& p) const {
    assert(isValid() && "Cannot transform point with uninitialized transform");
    return T_ * p;
  }

  // ========== Comparison ==========

  bool isApprox(const Transform_& other,
                Scalar tolerance = Scalar(1e-6)) const {
    return source_ == other.source_ && target_ == other.target_ &&
           T_.isApprox(other.T_, tolerance);
  }

  bool isIdentity(Scalar tolerance = Scalar(1e-6)) const {
    return source_ == target_ && T_.isApprox(SE3Type::Identity(), tolerance);
  }

  // ========== Interpolation ==========

  Transform_ slerp(const Transform_& other, Scalar t) const {
    if (source_ != other.source_ || target_ != other.target_) {
      throw std::runtime_error("slerp requires matching frames");
    }
    return Transform_(T_.slerp(other.T_, t), source_, target_);
  }

  std::string toString() const {
    std::stringstream ss;
    ss << "Transform [" << source_ << " -> " << target_ << "]";
    return ss.str();
  }
};

using Transformd = Transform_<double>;
using Transformf = Transform_<float>;
using Transform = Transformd;

}  // namespace nanopcl

#endif  // NANOPCL_TRANSFORM_TRANSFORM_HPP
