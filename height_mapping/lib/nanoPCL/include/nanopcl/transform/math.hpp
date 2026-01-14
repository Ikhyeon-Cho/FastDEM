// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Core transformation mathematics: angles, isometry construction,
// interpolation.

#ifndef NANOPCL_TRANSFORM_MATH_HPP
#define NANOPCL_TRANSFORM_MATH_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace npcl {

// =============================================================================
// Constants
// =============================================================================

namespace constants {

template <typename T>
inline constexpr T pi = T(3.14159265358979323846L);

}  // namespace constants

// =============================================================================
// Math Utilities
// =============================================================================

namespace math {

// =============================================================================
// Angle Conversions
// =============================================================================

/**
 * @brief Convert degrees to radians
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar deg2rad(Scalar deg) {
  return deg * constants::pi<Scalar> / Scalar(180);
}

/**
 * @brief Convert radians to degrees
 */
template <typename Scalar>
[[nodiscard]] constexpr Scalar rad2deg(Scalar rad) {
  return rad * Scalar(180) / constants::pi<Scalar>;
}

// =============================================================================
// RPY Extraction
// =============================================================================

/**
 * @brief Extract roll, pitch, yaw from rotation matrix (intrinsic ZYX
 * convention)
 * @param R 3x3 rotation matrix
 * @param roll Output roll angle (rotation around X-axis)
 * @param pitch Output pitch angle (rotation around Y-axis)
 * @param yaw Output yaw angle (rotation around Z-axis)
 */
template <typename Scalar>
void getRPY(const Eigen::Matrix<Scalar, 3, 3>& R, Scalar& roll, Scalar& pitch,
            Scalar& yaw) {
  pitch = std::asin(std::clamp(-R(2, 0), Scalar(-1), Scalar(1)));
  if (std::abs(std::cos(pitch)) > Scalar(1e-6)) {
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    // Gimbal lock: pitch is +-90 degrees
    roll = Scalar(0);
    yaw = std::atan2(-R(0, 1), R(1, 1));
  }
}

/**
 * @brief Extract roll angle from rotation matrix
 */
template <typename Scalar>
[[nodiscard]] Scalar getRoll(const Eigen::Matrix<Scalar, 3, 3>& R) {
  Scalar pitch = std::asin(std::clamp(-R(2, 0), Scalar(-1), Scalar(1)));
  if (std::abs(std::cos(pitch)) > Scalar(1e-6)) {
    return std::atan2(R(2, 1), R(2, 2));
  }
  return Scalar(0);  // Gimbal lock
}

/**
 * @brief Extract pitch angle from rotation matrix
 */
template <typename Scalar>
[[nodiscard]] Scalar getPitch(const Eigen::Matrix<Scalar, 3, 3>& R) {
  return std::asin(std::clamp(-R(2, 0), Scalar(-1), Scalar(1)));
}

/**
 * @brief Extract yaw angle from rotation matrix
 */
template <typename Scalar>
[[nodiscard]] Scalar getYaw(const Eigen::Matrix<Scalar, 3, 3>& R) {
  Scalar pitch = std::asin(std::clamp(-R(2, 0), Scalar(-1), Scalar(1)));
  if (std::abs(std::cos(pitch)) > Scalar(1e-6)) {
    return std::atan2(R(1, 0), R(0, 0));
  }
  return std::atan2(-R(0, 1), R(1, 1));  // Gimbal lock
}

// =============================================================================
// Isometry Construction
// =============================================================================

/**
 * @brief Create Isometry from roll-pitch-yaw angles (intrinsic ZYX convention)
 * @param roll Rotation around X-axis
 * @param pitch Rotation around Y-axis
 * @param yaw Rotation around Z-axis
 * @param translation Optional translation vector
 * @return Isometry transform
 */
template <typename Scalar>
[[nodiscard]] Eigen::Transform<Scalar, 3, Eigen::Isometry> isometryFromRPY(
    Scalar roll, Scalar pitch, Scalar yaw,
    const Eigen::Matrix<Scalar, 3, 1>& translation =
        Eigen::Matrix<Scalar, 3, 1>::Zero()) {
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using AngleAxis = Eigen::AngleAxis<Scalar>;
  using Quaternion = Eigen::Quaternion<Scalar>;

  Quaternion q = AngleAxis(yaw, Vector3::UnitZ()) *
                 AngleAxis(pitch, Vector3::UnitY()) *
                 AngleAxis(roll, Vector3::UnitX());

  Eigen::Transform<Scalar, 3, Eigen::Isometry> T;
  T.setIdentity();
  T.rotate(q);
  T.translation() = translation;
  return T;
}

/**
 * @brief Create Isometry from 2D pose (x, y, yaw)
 * @param x X position
 * @param y Y position
 * @param yaw Rotation around Z-axis
 * @return Isometry transform
 */
template <typename Scalar>
[[nodiscard]] Eigen::Transform<Scalar, 3, Eigen::Isometry> isometryFrom2D(
    Scalar x, Scalar y, Scalar yaw) {
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using AngleAxis = Eigen::AngleAxis<Scalar>;

  Eigen::Transform<Scalar, 3, Eigen::Isometry> T;
  T.setIdentity();
  T.translation() = Vector3(x, y, Scalar(0));
  T.rotate(AngleAxis(yaw, Vector3::UnitZ()));
  return T;
}

/**
 * @brief Create Isometry from quaternion components and translation
 * @param qx Quaternion x component
 * @param qy Quaternion y component
 * @param qz Quaternion z component
 * @param qw Quaternion w component
 * @param tx Translation x
 * @param ty Translation y
 * @param tz Translation z
 * @return Isometry transform
 */
template <typename Scalar>
[[nodiscard]] Eigen::Transform<Scalar, 3, Eigen::Isometry>
isometryFromQuaternion(Scalar qx, Scalar qy, Scalar qz, Scalar qw, Scalar tx,
                       Scalar ty, Scalar tz) {
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Quaternion = Eigen::Quaternion<Scalar>;

  Quaternion q(qw, qx, qy, qz);
  q.normalize();

  Eigen::Transform<Scalar, 3, Eigen::Isometry> T;
  T.setIdentity();
  T.rotate(q);
  T.translation() = Vector3(tx, ty, tz);
  return T;
}

/**
 * @brief Create Isometry from quaternion and translation vector
 */
template <typename Scalar>
[[nodiscard]] Eigen::Transform<Scalar, 3, Eigen::Isometry>
isometryFromQuaternion(const Eigen::Quaternion<Scalar>& q,
                       const Eigen::Matrix<Scalar, 3, 1>& translation =
                           Eigen::Matrix<Scalar, 3, 1>::Zero()) {
  Eigen::Transform<Scalar, 3, Eigen::Isometry> T;
  T.setIdentity();
  T.rotate(q.normalized());
  T.translation() = translation;
  return T;
}

// =============================================================================
// Interpolation
// =============================================================================

/**
 * @brief Spherical linear interpolation between two Isometry transforms
 * @param a Start transform
 * @param b End transform
 * @param t Interpolation parameter [0, 1]
 * @return Interpolated transform
 *
 * Translation is linearly interpolated, rotation uses quaternion slerp.
 */
template <typename Scalar>
[[nodiscard]] Eigen::Transform<Scalar, 3, Eigen::Isometry> slerp(
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& a,
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& b, Scalar t) {
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Quaternion = Eigen::Quaternion<Scalar>;

  // Linear interpolation for translation
  Vector3 trans = (Scalar(1) - t) * a.translation() + t * b.translation();

  // Spherical linear interpolation for rotation
  Quaternion q_a(a.rotation());
  Quaternion q_b(b.rotation());
  Quaternion q_interp = q_a.slerp(t, q_b);

  Eigen::Transform<Scalar, 3, Eigen::Isometry> result;
  result.setIdentity();
  result.rotate(q_interp);
  result.translation() = trans;
  return result;
}

}  // namespace math

}  // namespace npcl

#endif  // NANOPCL_TRANSFORM_MATH_HPP
