// nanoPCL - Core math utilities for 3D transformations
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_MATH_HPP
#define NANOPCL_CORE_MATH_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace nanopcl {

namespace constants {

template <typename T>
inline constexpr T pi = T(3.14159265358979323846L);

template <typename T>
inline constexpr T two_pi = T(6.28318530717958647692L);

template <typename T>
inline constexpr T half_pi = T(1.57079632679489661923L);

} // namespace constants

namespace math {

// Angle conversions
template <typename Scalar>
[[nodiscard]] constexpr Scalar deg2rad(Scalar deg) noexcept {
  return deg * constants::pi<Scalar> / Scalar(180);
}

template <typename Scalar>
[[nodiscard]] constexpr Scalar rad2deg(Scalar rad) noexcept {
  return rad * Scalar(180) / constants::pi<Scalar>;
}

template <typename Scalar>
[[nodiscard]] Scalar normalizeAngle(Scalar angle) noexcept {
  while (angle > constants::pi<Scalar>)
    angle -= constants::two_pi<Scalar>;
  while (angle < -constants::pi<Scalar>)
    angle += constants::two_pi<Scalar>;
  return angle;
}

// RPY extraction (Intrinsic ZYX: R = Rz(yaw) * Ry(pitch) * Rx(roll))
template <typename Scalar>
void getRPY(const Eigen::Matrix<Scalar, 3, 3>& R,
            Scalar& roll,
            Scalar& pitch,
            Scalar& yaw) {
  pitch = std::asin(std::clamp(-R(2, 0), Scalar(-1), Scalar(1)));

  if (std::abs(std::cos(pitch)) > Scalar(1e-6)) {
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw = std::atan2(R(1, 0), R(0, 0));
  } else {
    roll = Scalar(0);
    yaw = std::atan2(-R(0, 1), R(1, 1));
  }
}

template <typename Scalar>
[[nodiscard]] Scalar getRoll(const Eigen::Matrix<Scalar, 3, 3>& R) {
  Scalar pitch = std::asin(std::clamp(-R(2, 0), Scalar(-1), Scalar(1)));
  if (std::abs(std::cos(pitch)) > Scalar(1e-6)) {
    return std::atan2(R(2, 1), R(2, 2));
  }
  return Scalar(0);
}

template <typename Scalar>
[[nodiscard]] Scalar getPitch(const Eigen::Matrix<Scalar, 3, 3>& R) {
  return std::asin(std::clamp(-R(2, 0), Scalar(-1), Scalar(1)));
}

template <typename Scalar>
[[nodiscard]] Scalar getYaw(const Eigen::Matrix<Scalar, 3, 3>& R) {
  Scalar pitch = std::asin(std::clamp(-R(2, 0), Scalar(-1), Scalar(1)));
  if (std::abs(std::cos(pitch)) > Scalar(1e-6)) {
    return std::atan2(R(1, 0), R(0, 0));
  }
  return std::atan2(-R(0, 1), R(1, 1));
}

// Isometry factory functions
template <typename Scalar>
[[nodiscard]] Eigen::Transform<Scalar, 3, Eigen::Isometry> isometryFromRPY(
    Scalar roll,
    Scalar pitch,
    Scalar yaw,
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

template <typename Scalar>
[[nodiscard]] Eigen::Transform<Scalar, 3, Eigen::Isometry>
isometryFrom2D(Scalar x, Scalar y, Scalar yaw) {
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using AngleAxis = Eigen::AngleAxis<Scalar>;

  Eigen::Transform<Scalar, 3, Eigen::Isometry> T;
  T.setIdentity();
  T.translation() = Vector3(x, y, Scalar(0));
  T.rotate(AngleAxis(yaw, Vector3::UnitZ()));
  return T;
}

template <typename Scalar>
[[nodiscard]] Eigen::Transform<Scalar, 3, Eigen::Isometry>
isometryFromQuaternion(
    const Eigen::Quaternion<Scalar>& q,
    const Eigen::Matrix<Scalar, 3, 1>& translation =
        Eigen::Matrix<Scalar, 3, 1>::Zero()) {
  Eigen::Transform<Scalar, 3, Eigen::Isometry> T;
  T.setIdentity();
  T.rotate(q.normalized());
  T.translation() = translation;
  return T;
}

template <typename Scalar>
[[nodiscard]] Eigen::Transform<Scalar, 3, Eigen::Isometry>
isometryFromQuaternion(Scalar qx,
                       Scalar qy,
                       Scalar qz,
                       Scalar qw,
                       Scalar tx,
                       Scalar ty,
                       Scalar tz) {
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Quaternion = Eigen::Quaternion<Scalar>;
  return isometryFromQuaternion(Quaternion(qw, qx, qy, qz), Vector3(tx, ty, tz));
}

// Isometry interpolation (translation: lerp, rotation: slerp)
template <typename Scalar>
[[nodiscard]] Eigen::Transform<Scalar, 3, Eigen::Isometry>
slerp(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& a,
      const Eigen::Transform<Scalar, 3, Eigen::Isometry>& b,
      Scalar t) {
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Quaternion = Eigen::Quaternion<Scalar>;

  Vector3 trans = (Scalar(1) - t) * a.translation() + t * b.translation();

  Quaternion q_a(a.rotation());
  Quaternion q_b(b.rotation());
  Quaternion q_interp = q_a.slerp(t, q_b);

  Eigen::Transform<Scalar, 3, Eigen::Isometry> result;
  result.setIdentity();
  result.rotate(q_interp);
  result.translation() = trans;
  return result;
}

} // namespace math

} // namespace nanopcl

#endif // NANOPCL_CORE_MATH_HPP
