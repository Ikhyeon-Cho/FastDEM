// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_TRANSFORM_SE3_HPP
#define NANOPCL_TRANSFORM_SE3_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include "nanopcl/transform/so3.hpp"

namespace nanopcl {

/**
 * @brief SE(3) rigid transformation (rotation + translation)
 *
 * Represents rigid body transformation in SE(3) group using Eigen::Isometry.
 * Supports both double and float precision via template parameter.
 *
 * @tparam Scalar Floating point type (double or float)
 *
 * @note Users should use type aliases: SE3 (default double), SE3d, SE3f
 */
template <typename Scalar = double>
class SE3_ {
 public:
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Matrix4 = Eigen::Matrix<Scalar, 4, 4>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  using Isometry = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

 private:
  Isometry transform_;

  // Private helper functions
  static Matrix3 skew(const Vector3& v);
  static Matrix3 leftJacobian(const Vector3& omega);
  static Matrix3 leftJacobianInverse(const Vector3& omega);

 public:
  // ========== Constructors ==========

  SE3_() : transform_(Isometry::Identity()) {}
  explicit SE3_(const Isometry& tf) : transform_(tf) {}
  explicit SE3_(const Matrix4& matrix) : transform_(matrix) {}

  SE3_(const Vector3& translation, const SO3_<Scalar>& rotation) {
    transform_ = Isometry::Identity();
    transform_.translation() = translation;
    transform_.linear() = rotation.matrix();
  }

  SE3_(const Vector3& translation, const Matrix3& rotation) {
    transform_ = Isometry::Identity();
    transform_.translation() = translation;
    transform_.linear() = rotation;
  }

  SE3_(const Vector3& translation, const Quaternion& rotation) {
    transform_ = Isometry::Identity();
    transform_.translation() = translation;
    transform_.rotate(rotation);
  }

  // ========== Factory Methods ==========

  static SE3_ Identity() { return SE3_(); }

  static SE3_ fromTranslation(Scalar x, Scalar y, Scalar z) {
    SE3_ tf;
    tf.transform_.translation() = Vector3(x, y, z);
    return tf;
  }

  static SE3_ fromTranslation(const Vector3& translation) {
    SE3_ tf;
    tf.transform_.translation() = translation;
    return tf;
  }

  /// Create SE3 from 2D pose (x, y, theta)
  static SE3_ from2D(Scalar x, Scalar y, Scalar theta);

  /// Create SE3 from roll-pitch-yaw angles
  static SE3_ fromRPY(Scalar roll, Scalar pitch, Scalar yaw);

  /// Create SE3 from quaternion components
  static SE3_ fromQuaternion(Scalar qx, Scalar qy, Scalar qz, Scalar qw,
                             const Vector3& translation = Vector3::Zero());

  /// Create SE3 from quaternion
  static SE3_ fromQuaternion(const Quaternion& q,
                             const Vector3& translation = Vector3::Zero());

  /// Create SE3 from SO3 rotation
  static SE3_ fromRotation(const SO3_<Scalar>& rotation,
                           const Vector3& translation = Vector3::Zero());

  // ========== Lie Algebra Operations ==========

  /// Exponential map: se(3) -> SE(3)
  static SE3_ exp(const Vector6& xi);

  /// Logarithmic map: SE(3) -> se(3)
  Vector6 log() const;

  // ========== Accessors ==========

  Matrix4 matrix() const { return transform_.matrix(); }

  SO3_<Scalar> rotation() const {
    return SO3_<Scalar>::fromRotationMatrix(transform_.rotation());
  }

  Vector3 translation() const { return transform_.translation(); }

  Quaternion quaternion() const { return Quaternion(transform_.rotation()); }

  const Isometry& isometry() const { return transform_; }
  Isometry& isometry() { return transform_; }

  Scalar x() const { return transform_.translation().x(); }
  Scalar y() const { return transform_.translation().y(); }
  Scalar z() const { return transform_.translation().z(); }

  /// Get roll, pitch, yaw angles
  void getRPY(Scalar& roll, Scalar& pitch, Scalar& yaw) const;

  Scalar yaw() const {
    auto m = transform_.rotation();
    return std::atan2(m(1, 0), m(0, 0));
  }

  // ========== Setters ==========

  void setTranslation(const Vector3& translation) {
    transform_.translation() = translation;
  }

  void setRotation(const SO3_<Scalar>& rotation) {
    transform_.linear() = rotation.matrix();
  }

  void setRotation(const Matrix3& rotation) { transform_.linear() = rotation; }

  void setRotation(const Quaternion& rotation) {
    transform_.linear() = rotation.toRotationMatrix();
  }

  // ========== Transform Operations ==========

  Vector3 operator*(const Vector3& p) const { return transform_ * p; }

  template <typename OtherScalar>
  Eigen::Matrix<OtherScalar, 3, 1> operator*(
      const Eigen::Matrix<OtherScalar, 3, 1>& p) const {
    return (transform_ * p.template cast<Scalar>()).template cast<OtherScalar>();
  }

  SE3_ operator*(const SE3_& other) const {
    return SE3_(transform_ * other.transform_);
  }

  SE3_& operator*=(const SE3_& other) {
    transform_ = transform_ * other.transform_;
    return *this;
  }

  SE3_ inverse() const { return SE3_(transform_.inverse()); }

  // ========== Conversion ==========

  template <typename NewScalar>
  SE3_<NewScalar> cast() const {
    return SE3_<NewScalar>(transform_.template cast<NewScalar>());
  }

  // ========== Interpolation ==========

  /// Spherical linear interpolation
  SE3_ slerp(const SE3_& other, Scalar t) const;

  // ========== Comparison ==========

  bool isApprox(const SE3_& other, Scalar tolerance = Scalar(1e-6)) const {
    return transform_.isApprox(other.transform_, tolerance);
  }

  bool isIdentity(Scalar tolerance = Scalar(1e-6)) const {
    return transform_.isApprox(Isometry::Identity(), tolerance);
  }

  /// String representation
  std::string toString() const;
};

// ========== Type Aliases ==========
using SE3d = SE3_<double>;
using SE3f = SE3_<float>;
using SE3 = SE3d;

}  // namespace nanopcl

// Include implementation
#include "nanopcl/transform/impl/se3_impl.hpp"

#endif  // NANOPCL_TRANSFORM_SE3_HPP
