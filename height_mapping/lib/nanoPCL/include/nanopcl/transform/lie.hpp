// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Lie algebra operations for SO(3) and SE(3).
// Include this header for optimization/SLAM applications.

#ifndef NANOPCL_TRANSFORM_LIE_HPP
#define NANOPCL_TRANSFORM_LIE_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>

namespace npcl {

// =============================================================================
// Skew-Symmetric Matrix
// =============================================================================

/**
 * @brief Create skew-symmetric matrix from 3-vector
 * @param v Input vector
 * @return 3x3 skew-symmetric matrix such that skew(v) * u = v x u
 */
template <typename Scalar>
[[nodiscard]] Eigen::Matrix<Scalar, 3, 3> skew(
    const Eigen::Matrix<Scalar, 3, 1>& v) {
  Eigen::Matrix<Scalar, 3, 3> s;
  // clang-format off
  s << Scalar(0), -v.z(),  v.y(),
        v.z(),  Scalar(0), -v.x(),
       -v.y(),  v.x(),  Scalar(0);
  // clang-format on
  return s;
}

// =============================================================================
// Left Jacobian of SO(3)
// =============================================================================

/**
 * @brief Left Jacobian of SO(3)
 * @param omega Angular velocity vector (axis * angle)
 * @return 3x3 left Jacobian matrix
 *
 * The left Jacobian J_l relates small perturbations in the tangent space:
 * exp(omega + delta) ≈ exp(J_l * delta) * exp(omega)
 */
template <typename Scalar>
[[nodiscard]] Eigen::Matrix<Scalar, 3, 3> leftJacobian(
    const Eigen::Matrix<Scalar, 3, 1>& omega) {
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

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

/**
 * @brief Inverse of Left Jacobian of SO(3)
 * @param omega Angular velocity vector (axis * angle)
 * @return 3x3 inverse left Jacobian matrix
 */
template <typename Scalar>
[[nodiscard]] Eigen::Matrix<Scalar, 3, 3> leftJacobianInverse(
    const Eigen::Matrix<Scalar, 3, 1>& omega) {
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

  Scalar theta = omega.norm();
  if (theta < Scalar(1e-10)) {
    return Matrix3::Identity();
  }

  Vector3 axis = omega / theta;
  Matrix3 omega_hat = skew(axis);
  Scalar half_theta = Scalar(0.5) * theta;
  Scalar c =
      Scalar(1) / (theta * theta) -
      (Scalar(1) + std::cos(theta)) / (Scalar(2) * theta * std::sin(theta));

  return Matrix3::Identity() - half_theta * omega_hat +
         c * omega_hat * omega_hat;
}

// =============================================================================
// SO(3) Exponential and Logarithmic Maps
// =============================================================================

/**
 * @brief SO(3) exponential map: so(3) -> SO(3)
 * @param omega Angular velocity vector (axis * angle)
 * @return Rotation matrix
 *
 * Computes R = exp([omega]_x) using Rodrigues' formula.
 */
template <typename Scalar>
[[nodiscard]] Eigen::Matrix<Scalar, 3, 3> so3Exp(
    const Eigen::Matrix<Scalar, 3, 1>& omega) {
  using AngleAxis = Eigen::AngleAxis<Scalar>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

  Scalar angle = omega.norm();
  if (angle < Scalar(1e-10)) {
    return Eigen::Matrix<Scalar, 3, 3>::Identity();
  }

  Vector3 axis = omega / angle;
  return AngleAxis(angle, axis).toRotationMatrix();
}

/**
 * @brief SO(3) logarithmic map: SO(3) -> so(3)
 * @param R Rotation matrix
 * @return Angular velocity vector (axis * angle)
 *
 * Computes omega such that R = exp([omega]_x).
 */
template <typename Scalar>
[[nodiscard]] Eigen::Matrix<Scalar, 3, 1> so3Log(
    const Eigen::Matrix<Scalar, 3, 3>& R) {
  using AngleAxis = Eigen::AngleAxis<Scalar>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

  AngleAxis aa(R);
  Scalar angle = aa.angle();
  if (angle < Scalar(1e-10)) {
    return Vector3::Zero();
  }
  return angle * aa.axis();
}

// =============================================================================
// SE(3) Exponential and Logarithmic Maps
// =============================================================================

/**
 * @brief SE(3) exponential map: se(3) -> SE(3)
 * @param xi 6-vector [v; omega] where v is linear velocity, omega is angular
 * @return Isometry transform
 *
 * Computes T = exp([xi]^) where [xi]^ is the 4x4 se(3) matrix.
 */
template <typename Scalar>
[[nodiscard]] Eigen::Transform<Scalar, 3, Eigen::Isometry> se3Exp(
    const Eigen::Matrix<Scalar, 6, 1>& xi) {
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

  Vector3 v = xi.template head<3>();
  Vector3 omega = xi.template tail<3>();

  Matrix3 R = so3Exp(omega);
  Matrix3 V = leftJacobian(omega);
  Vector3 t = V * v;

  Eigen::Transform<Scalar, 3, Eigen::Isometry> T;
  T.setIdentity();
  T.linear() = R;
  T.translation() = t;
  return T;
}

/**
 * @brief SE(3) logarithmic map: SE(3) -> se(3)
 * @param T Isometry transform
 * @return 6-vector [v; omega]
 *
 * Computes xi such that T = exp([xi]^).
 */
template <typename Scalar>
[[nodiscard]] Eigen::Matrix<Scalar, 6, 1> se3Log(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& T) {
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vector6 = Eigen::Matrix<Scalar, 6, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

  Matrix3 R = T.rotation();
  Vector3 omega = so3Log(R);
  Matrix3 V_inv = leftJacobianInverse(omega);
  Vector3 v = V_inv * T.translation();

  Vector6 xi;
  xi << v, omega;
  return xi;
}

// =============================================================================
// Convenience Overloads (double precision)
// =============================================================================

/// SE(3) exp for double precision (convenience)
[[nodiscard]] inline Eigen::Isometry3d se3Exp(
    const Eigen::Matrix<double, 6, 1>& xi) {
  return se3Exp<double>(xi);
}

/// SE(3) log for double precision (convenience)
[[nodiscard]] inline Eigen::Matrix<double, 6, 1> se3Log(
    const Eigen::Isometry3d& T) {
  return se3Log<double>(T);
}

/// SE(3) exp for float precision (convenience)
[[nodiscard]] inline Eigen::Isometry3f se3Expf(
    const Eigen::Matrix<float, 6, 1>& xi) {
  return se3Exp<float>(xi);
}

/// SE(3) log for float precision (convenience)
[[nodiscard]] inline Eigen::Matrix<float, 6, 1> se3Logf(
    const Eigen::Isometry3f& T) {
  return se3Log<float>(T);
}

}  // namespace npcl

#endif  // NANOPCL_TRANSFORM_LIE_HPP
