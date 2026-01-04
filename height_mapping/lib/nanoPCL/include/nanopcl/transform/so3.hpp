// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_TRANSFORM_SO3_HPP
#define NANOPCL_TRANSFORM_SO3_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

namespace nanopcl {

/**
 * @brief SO(3) rotation transformation
 *
 * Represents 3D rotation in SO(3) special orthogonal group.
 * Uses Eigen::Quaternion internally for efficiency and numerical stability.
 * Supports both double and float precision via template parameter.
 *
 * @tparam Scalar Floating point type (double or float)
 *
 * @note Users should use type aliases: SO3 (default double), SO3d, SO3f
 */
template <typename Scalar = double>
class SO3_ {
 public:
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using AngleAxis = Eigen::AngleAxis<Scalar>;

 private:
  Quaternion rotation_;

 public:
  // ========== Constructors ==========

  /// Default constructor (identity rotation)
  SO3_() : rotation_(Quaternion::Identity()) {}

  /// Construct from Eigen::Quaternion
  explicit SO3_(const Quaternion& q) : rotation_(q.normalized()) {}

  /// Construct from rotation matrix
  explicit SO3_(const Matrix3& R) : rotation_(R) { rotation_.normalize(); }

  // ========== Factory Methods ==========

  /// Create identity rotation
  static SO3_ Identity() { return SO3_(); }

  /// Create from roll-pitch-yaw angles (intrinsic ZYX convention)
  static SO3_ fromRPY(Scalar roll, Scalar pitch, Scalar yaw);

  /// Create from yaw angle only (rotation around Z-axis)
  static SO3_ fromYaw(Scalar yaw) {
    return SO3_(Quaternion(AngleAxis(yaw, Vector3::UnitZ())));
  }

  /// Create from quaternion components (x, y, z, w)
  static SO3_ fromQuaternion(Scalar x, Scalar y, Scalar z, Scalar w) {
    return SO3_(Quaternion(w, x, y, z));
  }

  /// Create from Eigen quaternion
  static SO3_ fromQuaternion(const Quaternion& q) { return SO3_(q); }

  /// Create from angle-axis representation
  static SO3_ fromAngleAxis(Scalar angle, const Vector3& axis);

  /// Create from rotation matrix
  static SO3_ fromRotationMatrix(const Matrix3& R) { return SO3_(R); }

  // ========== Lie Algebra Operations ==========

  /// Exponential map: so(3) -> SO(3)
  static SO3_ exp(const Vector3& omega);

  /// Logarithm map: SO(3) -> so(3)
  Vector3 log() const;

  // ========== Accessors ==========

  const Quaternion& quaternion() const { return rotation_; }
  Matrix3 matrix() const { return rotation_.toRotationMatrix(); }

  /// Get roll, pitch, yaw angles
  void getRPY(Scalar& roll, Scalar& pitch, Scalar& yaw) const;

  Scalar yaw() const {
    auto m = matrix();
    return std::atan2(m(1, 0), m(0, 0));
  }

  // ========== Operations ==========

  SO3_ inverse() const { return SO3_(rotation_.conjugate()); }

  Vector3 operator*(const Vector3& v) const { return rotation_ * v; }

  SO3_ operator*(const SO3_& other) const {
    return SO3_(rotation_ * other.rotation_);
  }

  SO3_& operator*=(const SO3_& other) {
    rotation_ = rotation_ * other.rotation_;
    return *this;
  }

  // ========== Interpolation ==========

  SO3_ slerp(const SO3_& other, Scalar t) const {
    return SO3_(rotation_.slerp(t, other.rotation_));
  }

  // ========== Comparison ==========

  bool isApprox(const SO3_& other, Scalar tolerance = Scalar(1e-6)) const {
    return rotation_.isApprox(other.rotation_, tolerance);
  }

  bool isIdentity(Scalar tolerance = Scalar(1e-6)) const {
    return isApprox(SO3_::Identity(), tolerance);
  }

  /// String representation
  std::string toString() const;
};

// ========== Type Aliases ==========
using SO3d = SO3_<double>;
using SO3f = SO3_<float>;
using SO3 = SO3d;

// ========== Utility Functions ==========

template <typename Scalar>
inline Scalar deg2rad(Scalar deg) {
  return deg * Scalar(M_PI) / Scalar(180);
}

template <typename Scalar>
inline Scalar rad2deg(Scalar rad) {
  return rad * Scalar(180) / Scalar(M_PI);
}

template <typename Scalar>
inline SO3_<Scalar> slerp(const SO3_<Scalar>& R_start,
                          const SO3_<Scalar>& R_end, Scalar t) {
  return R_start.slerp(R_end, t);
}

}  // namespace nanopcl

// Include implementation
#include "nanopcl/transform/impl/so3_impl.hpp"

#endif  // NANOPCL_TRANSFORM_SO3_HPP
