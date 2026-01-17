// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Frame-aware rigid body transformation with TF2 semantics.

#ifndef NANOPCL_TRANSFORM_TRANSFORM_HPP
#define NANOPCL_TRANSFORM_TRANSFORM_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>

#include "nanopcl/core/timestamp.hpp"

namespace npcl {

/**
 * @brief Frame-aware rigid body transformation
 *
 * Represents a rigid body transformation from child frame to parent frame.
 * Uses Eigen::Isometry internally with explicit frame semantics.
 *
 * TF2 Convention:
 * - T_parent_child transforms points from child to parent frame
 * - childFrame(): source frame (sensor, base_link)
 * - parentFrame(): target frame (map, odom, world)
 *
 * @tparam Scalar Floating point type (double or float)
 */
template <typename Scalar = double>
class Transform_ {
 public:
  using Isometry = Eigen::Transform<Scalar, 3, Eigen::Isometry>;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
  using Quaternion = Eigen::Quaternion<Scalar>;

 private:
  Isometry T_{Isometry::Identity()};
  std::string parent_;
  std::string child_;
  Timestamp stamp_{0};

 public:
  // ===========================================================================
  // Constructors
  // ===========================================================================

  /// @brief Default constructor (creates invalid transform)
  Transform_() = default;

  /**
   * @brief Construct a transform between two frames
   * @param parent Parent frame name (target frame)
   * @param child Child frame name (source frame)
   * @param T Isometry transform (default: Identity)
   */
  Transform_(std::string parent, std::string child,
             const Isometry& T = Isometry::Identity())
      : T_(T), parent_(std::move(parent)), child_(std::move(child)) {}

  /**
   * @brief Construct from Eigen::Transform with any mode
   * @tparam Mode Eigen transform mode (Isometry or Affine)
   */
  template <int Mode>
  Transform_(std::string parent, std::string child,
             const Eigen::Transform<Scalar, 3, Mode>& T)
      : parent_(std::move(parent)), child_(std::move(child)) {
    static_assert(Mode == Eigen::Isometry || Mode == Eigen::Affine,
                  "Only Isometry or Affine transforms are supported");
    T_.matrix() = T.matrix();
  }

  // ===========================================================================
  // Factory Methods
  // ===========================================================================

  /**
   * @brief Create an identity transform within a single frame
   * @param frame Frame name (used for both child and parent)
   */
  [[nodiscard]] static Transform_ identity(const std::string& frame) {
    return Transform_(frame, frame, Isometry::Identity());
  }

  /**
   * @brief Create transform from roll-pitch-yaw angles
   * @param parent Parent frame name (target frame)
   * @param child Child frame name (source frame)
   * @param roll Rotation around X-axis (radians)
   * @param pitch Rotation around Y-axis (radians)
   * @param yaw Rotation around Z-axis (radians)
   * @param translation Translation vector (default: zero)
   */
  [[nodiscard]] static Transform_ fromRPY(
      const std::string& parent, const std::string& child, Scalar roll,
      Scalar pitch, Scalar yaw, const Vector3& translation = Vector3::Zero());

  /**
   * @brief Create transform from roll-pitch-yaw vector
   * @param parent Parent frame name (target frame)
   * @param child Child frame name (source frame)
   * @param rpy Vector3 containing (roll, pitch, yaw) in radians
   * @param translation Translation vector (default: zero)
   */
  [[nodiscard]] static Transform_ fromRPY(
      const std::string& parent, const std::string& child, const Vector3& rpy,
      const Vector3& translation = Vector3::Zero()) {
    return fromRPY(parent, child, rpy.x(), rpy.y(), rpy.z(), translation);
  }

  /**
   * @brief Create transform from 2D pose
   * @param parent Parent frame name (target frame)
   * @param child Child frame name (source frame)
   * @param x X position
   * @param y Y position
   * @param yaw Rotation around Z-axis (radians)
   */
  [[nodiscard]] static Transform_ from2D(const std::string& parent,
                                         const std::string& child, Scalar x,
                                         Scalar y, Scalar yaw);

  // ===========================================================================
  // Validation
  // ===========================================================================

  /// @brief Check if transform is properly initialized
  [[nodiscard]] bool isValid() const noexcept {
    return !child_.empty() && !parent_.empty();
  }

  // ===========================================================================
  // Frame Accessors
  // ===========================================================================

  /// @brief Get parent frame name (target frame)
  [[nodiscard]] const std::string& parentFrame() const noexcept {
    return parent_;
  }

  /// @brief Get child frame name (source frame)
  [[nodiscard]] const std::string& childFrame() const noexcept {
    return child_;
  }

  /// @brief Get timestamp (nanoseconds since epoch)
  [[nodiscard]] Timestamp timestamp() const noexcept { return stamp_; }

  /// @brief Set timestamp (nanoseconds since epoch)
  void setTimestamp(Timestamp ts) noexcept { stamp_ = ts; }

  // ===========================================================================
  // Geometry Accessors
  // ===========================================================================

  /// @brief Get underlying Isometry transform
  [[nodiscard]] const Isometry& isometry() const noexcept { return T_; }

  /// @brief Get translation vector
  [[nodiscard]] Vector3 translation() const noexcept { return T_.translation(); }

  /// @brief Get rotation matrix
  [[nodiscard]] Matrix3 rotation() const { return T_.rotation(); }

  /// @brief Get rotation as quaternion
  [[nodiscard]] Quaternion quaternion() const { return Quaternion(T_.rotation()); }

  /// @brief Get X translation component
  [[nodiscard]] Scalar x() const noexcept { return T_.translation().x(); }

  /// @brief Get Y translation component
  [[nodiscard]] Scalar y() const noexcept { return T_.translation().y(); }

  /// @brief Get Z translation component
  [[nodiscard]] Scalar z() const noexcept { return T_.translation().z(); }

  /// @brief Get roll angle (rotation around X-axis)
  [[nodiscard]] Scalar roll() const;

  /// @brief Get pitch angle (rotation around Y-axis)
  [[nodiscard]] Scalar pitch() const;

  /// @brief Get yaw angle (rotation around Z-axis)
  [[nodiscard]] Scalar yaw() const;

  /// @brief Get roll, pitch, yaw angles
  void getRPY(Scalar& roll, Scalar& pitch, Scalar& yaw_out) const;

  /// @brief Get translation in float precision
  [[nodiscard]] Eigen::Vector3f translationf() const {
    return translation().template cast<float>();
  }

  /// @brief Get quaternion in float precision
  [[nodiscard]] Eigen::Quaternionf quaternionf() const {
    return quaternion().template cast<float>();
  }

  // ===========================================================================
  // Transform Operations
  // ===========================================================================

  /**
   * @brief Get inverse transform
   *
   * Swaps child and parent frames, inverts the Isometry, preserves timestamp.
   * @throws std::runtime_error if transform is not initialized
   */
  [[nodiscard]] Transform_ inverse() const {
    if (!isValid()) {
      throw std::runtime_error("Cannot invert uninitialized transform");
    }
    Transform_ result(child_, parent_, T_.inverse());
    result.setTimestamp(stamp_);
    return result;
  }

  /**
   * @brief Compose two transforms (frame-checked)
   *
   * Computes: `T_parent_other = T_parent_child * T_child_other`
   *
   * This operation represents frame chaining. The child frame of the first transform 
   * MUST match the parent frame of the second transform.
   *
   * @param other The next transform in the chain
   * @return Composed transform with merged frame information
   *
   * @throws std::runtime_error if frame mismatch or transforms are uninitialized.
   *
   * @code
   *   Transform T_map_base("map", "base_link", isometry1);
   *   Transform T_base_lidar("base_link", "lidar", isometry2);
   *
   *   // Success: "map" -> "base_link" -> "lidar"
   *   Transform T_map_lidar = T_map_base * T_base_lidar;
   *
   *   // Error: "lidar" does not match "map"
   *   Transform invalid = T_base_lidar * T_map_base;
   * @endcode
   */
  [[nodiscard]] Transform_ operator*(const Transform_& other) const {
    if (!isValid() || !other.isValid()) {
      throw std::runtime_error("Cannot compose uninitialized transforms");
    }
    if (child_ != other.parent_) {
      throw std::runtime_error("Frame mismatch: '" + child_ + "' != '" +
                               other.parent_ + "'");
    }
    Isometry composed;
    composed.matrix() = T_.matrix() * other.T_.matrix();
    Transform_ result(parent_, other.child_, composed);
    result.setTimestamp(std::max(stamp_, other.stamp_));
    return result;
  }

  /// @brief In-place composition (frame-checked)
  Transform_& operator*=(const Transform_& other) {
    *this = *this * other;
    return *this;
  }

  /**
   * @brief Transform a point from child frame to parent frame
   * @tparam T Point scalar type
   * @param p Point in child frame
   * @return Transformed point in parent frame
   */
  template <typename T>
  [[nodiscard]] Eigen::Matrix<T, 3, 1> operator*(
      const Eigen::Matrix<T, 3, 1>& p) const {
    if (!isValid()) {
      throw std::runtime_error("Cannot apply uninitialized transform");
    }
    return T_ * p.template cast<Scalar>();
  }

  // ===========================================================================
  // Comparison
  // ===========================================================================

  /**
   * @brief Check if two transforms are approximately equal
   * @param other Transform to compare
   * @param tol Tolerance for comparison
   */
  [[nodiscard]] bool isApprox(const Transform_& other,
                              Scalar tol = Scalar(1e-6)) const {
    return parent_ == other.parent_ && child_ == other.child_ &&
           T_.isApprox(other.T_, tol);
  }

  /**
   * @brief Check if transform is approximately identity
   * @param tol Tolerance for comparison
   */
  [[nodiscard]] bool isIdentity(Scalar tol = Scalar(1e-6)) const {
    return parent_ == child_ && T_.isApprox(Isometry::Identity(), tol);
  }

  // ===========================================================================
  // Interpolation
  // ===========================================================================

  /**
   * @brief Spherical linear interpolation between transforms
   * @param other End transform (must have matching frames)
   * @param t Interpolation parameter [0, 1]
   * @return Interpolated transform with linearly interpolated timestamp
   * @throws std::runtime_error if frames don't match
   */
  [[nodiscard]] Transform_ slerp(const Transform_& other, Scalar t) const;
};

// =============================================================================
// Type Aliases
// =============================================================================

using Transformd = Transform_<double>;
using Transformf = Transform_<float>;
using Transform = Transformd;

}  // namespace npcl

#include "nanopcl/transform/impl/transform_impl.hpp"

#endif  // NANOPCL_TRANSFORM_TRANSFORM_HPP
