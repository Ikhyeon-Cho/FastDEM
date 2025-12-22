// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_TRANSFORM_SE3_HPP
#define NANOPCL_TRANSFORM_SE3_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sstream>
#include <string>

#include "nanopcl/transform/so3.hpp"

namespace nanopcl {

// ============================================================================
// SE(3) - Special Euclidean Group (Rotation + Translation)
// ============================================================================

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

  static SE3_ from2D(Scalar x, Scalar y, Scalar theta) {
    SE3_ tf;
    tf.transform_.translation() = Vector3(x, y, 0);
    tf.transform_.rotate(Eigen::AngleAxis<Scalar>(theta, Vector3::UnitZ()));
    return tf;
  }

  static SE3_ fromRPY(Scalar roll, Scalar pitch, Scalar yaw) {
    Eigen::AngleAxis<Scalar> rollAngle(roll, Vector3::UnitX());
    Eigen::AngleAxis<Scalar> pitchAngle(pitch, Vector3::UnitY());
    Eigen::AngleAxis<Scalar> yawAngle(yaw, Vector3::UnitZ());

    Quaternion q = yawAngle * pitchAngle * rollAngle;
    SE3_ tf;
    tf.transform_.rotate(q);
    return tf;
  }

  static SE3_ fromQuaternion(Scalar qx, Scalar qy, Scalar qz, Scalar qw,
                             const Vector3& translation = Vector3::Zero()) {
    Quaternion q(qw, qx, qy, qz);
    return SE3_(translation, q);
  }

  static SE3_ fromQuaternion(const Quaternion& q,
                             const Vector3& translation = Vector3::Zero()) {
    return SE3_(translation, q);
  }

  static SE3_ fromRotation(const SO3_<Scalar>& rotation,
                           const Vector3& translation = Vector3::Zero()) {
    return SE3_(translation, rotation.matrix());
  }

  // ========== Lie Algebra Operations ==========

 private:
  static Matrix3 skew(const Vector3& v) {
    Matrix3 s;
    s << Scalar(0), -v.z(), v.y(),
         v.z(), Scalar(0), -v.x(),
         -v.y(), v.x(), Scalar(0);
    return s;
  }

  static Matrix3 leftJacobian(const Vector3& omega) {
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

  static Matrix3 leftJacobianInverse(const Vector3& omega) {
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

 public:
  static SE3_ exp(const Vector6& xi) {
    Vector3 v = xi.template head<3>();
    Vector3 omega = xi.template tail<3>();
    SO3_<Scalar> R = SO3_<Scalar>::exp(omega);
    Matrix3 V = leftJacobian(omega);
    Vector3 t = V * v;
    return SE3_(t, R);
  }

  Vector6 log() const {
    SO3_<Scalar> R = rotation();
    Vector3 omega = R.log();
    Matrix3 V_inv = leftJacobianInverse(omega);
    Vector3 v = V_inv * translation();
    Vector6 xi;
    xi << v, omega;
    return xi;
  }

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

  void getRPY(Scalar& roll, Scalar& pitch, Scalar& yaw) const {
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

  // ========== Interpolation ==========

  SE3_ slerp(const SE3_& other, Scalar t) const {
    Vector3 trans = (Scalar(1) - t) * translation() + t * other.translation();
    SO3_<Scalar> rot = rotation().slerp(other.rotation(), t);
    return SE3_(trans, rot);
  }

  // ========== Comparison ==========

  bool isApprox(const SE3_& other, Scalar tolerance = Scalar(1e-6)) const {
    return transform_.isApprox(other.transform_, tolerance);
  }

  bool isIdentity(Scalar tolerance = Scalar(1e-6)) const {
    return transform_.isApprox(Isometry::Identity(), tolerance);
  }

  std::string toString() const {
    std::stringstream ss;
    ss << "SE3(t: [" << x() << ", " << y() << ", " << z() << "], ";
    Scalar roll, pitch, yaw;
    getRPY(roll, pitch, yaw);
    ss << "RPY: [" << roll << ", " << pitch << ", " << yaw << "])";
    return ss.str();
  }
};

// ========== Type Aliases ==========
using SE3d = SE3_<double>;
using SE3f = SE3_<float>;
using SE3 = SE3d;

}  // namespace nanopcl

#endif  // NANOPCL_TRANSFORM_SE3_HPP
