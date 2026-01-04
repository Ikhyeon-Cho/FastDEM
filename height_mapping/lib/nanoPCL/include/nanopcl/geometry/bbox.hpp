// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_GEOMETRY_BBOX_HPP
#define NANOPCL_GEOMETRY_BBOX_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <limits>
#include <stdexcept>
#include <string>

#include "nanopcl/core/point.hpp"
#include "nanopcl/core/timestamp.hpp"

namespace nanopcl {

// ============================================================================
// AxisAlignedBox - Axis-Aligned Bounding Box
// ============================================================================

/**
 * @brief Axis-Aligned Bounding Box in 3D with frame semantics
 */
class AxisAlignedBox {
 public:
  using Vector3 = Eigen::Vector3f;

 private:
  Point min_;
  Point max_;
  std::string frame_id_;
  Timestamp timestamp_ns_{0};

 public:
  AxisAlignedBox()
      : min_(Vector3::Constant(std::numeric_limits<float>::max())),
        max_(Vector3::Constant(std::numeric_limits<float>::lowest())) {}

  AxisAlignedBox(const Point& min, const Point& max,
                 const std::string& frame_id = "", Timestamp timestamp = 0)
      : min_(min), max_(max), frame_id_(frame_id), timestamp_ns_(timestamp) {}

  // ========== Factory Methods ==========

  static AxisAlignedBox fromMinMax(const Point& min, const Point& max,
                                   const std::string& frame_id = "",
                                   Timestamp timestamp = 0) {
    return AxisAlignedBox(min, max, frame_id, timestamp);
  }

  static AxisAlignedBox fromCenterSize(const Point& center, const Vector3& size,
                                       const std::string& frame_id = "",
                                       Timestamp timestamp = 0) {
    Vector3 half_size = size / 2.0f;
    return AxisAlignedBox(center - half_size, center + half_size, frame_id,
                          timestamp);
  }

  // ========== Accessors ==========

  const Point& min() const noexcept { return min_; }
  const Point& max() const noexcept { return max_; }
  Point center() const noexcept { return (min_ + max_) / 2.0f; }
  Vector3 size() const noexcept { return max_ - min_; }
  Vector3 extent() const noexcept { return (max_ - min_) / 2.0f; }

  const std::string& frameId() const noexcept { return frame_id_; }
  void setFrameId(const std::string& id) { frame_id_ = id; }
  Timestamp timestamp() const noexcept { return timestamp_ns_; }
  void setTimestamp(Timestamp ts) noexcept { timestamp_ns_ = ts; }

  // ========== Query Methods ==========

  bool isEmpty() const noexcept {
    return min_.x() > max_.x() || min_.y() > max_.y() || min_.z() > max_.z();
  }

  float volume() const noexcept {
    if (isEmpty()) return 0.0f;
    Vector3 s = size();
    return s.x() * s.y() * s.z();
  }

  bool contains(const Point& point) const noexcept {
    return (point.x() >= min_.x() && point.x() <= max_.x() &&
            point.y() >= min_.y() && point.y() <= max_.y() &&
            point.z() >= min_.z() && point.z() <= max_.z());
  }

  std::array<Point, 8> corners() const {
    return {{Point(min_.x(), min_.y(), min_.z()),
             Point(max_.x(), min_.y(), min_.z()),
             Point(min_.x(), max_.y(), min_.z()),
             Point(max_.x(), max_.y(), min_.z()),
             Point(min_.x(), min_.y(), max_.z()),
             Point(max_.x(), min_.y(), max_.z()),
             Point(min_.x(), max_.y(), max_.z()),
             Point(max_.x(), max_.y(), max_.z())}};
  }
};

using AABB = AxisAlignedBox;

// ============================================================================
// OrientedBox - Oriented Bounding Box
// ============================================================================

/**
 * @brief Oriented Bounding Box in 3D with frame semantics
 */
class OrientedBox {
 public:
  using Vector3 = Eigen::Vector3f;
  using Matrix3 = Eigen::Matrix3f;
  using Quaternion = Eigen::Quaternionf;

 private:
  Point center_;
  Quaternion orientation_;
  Vector3 extent_;
  std::string frame_id_;
  Timestamp timestamp_ns_{0};

 public:
  OrientedBox()
      : center_(Vector3::Zero()),
        orientation_(Quaternion::Identity()),
        extent_(Vector3::Zero()) {}

  OrientedBox(const Point& center, const Quaternion& orientation,
              const Vector3& extent, const std::string& frame_id = "",
              Timestamp timestamp = 0)
      : center_(center),
        orientation_(orientation.normalized()),
        extent_(extent),
        frame_id_(frame_id),
        timestamp_ns_(timestamp) {}

  // ========== Factory Methods ==========

  static OrientedBox fromAABB(const AxisAlignedBox& aabb) {
    return OrientedBox(aabb.center(), Quaternion::Identity(), aabb.extent(),
                       aabb.frameId(), aabb.timestamp());
  }

  static OrientedBox fromCenterSize(const Point& center, const Vector3& size,
                                    const std::string& frame_id = "",
                                    Timestamp timestamp = 0) {
    return OrientedBox(center, Quaternion::Identity(), size / 2.0f, frame_id,
                       timestamp);
  }

  // ========== Accessors ==========

  const Point& center() const noexcept { return center_; }
  const Quaternion& orientation() const noexcept { return orientation_; }
  Matrix3 rotationMatrix() const noexcept { return orientation_.toRotationMatrix(); }
  const Vector3& extent() const noexcept { return extent_; }
  Vector3 size() const noexcept { return extent_ * 2.0f; }

  const std::string& frameId() const noexcept { return frame_id_; }
  void setFrameId(const std::string& id) { frame_id_ = id; }
  Timestamp timestamp() const noexcept { return timestamp_ns_; }
  void setTimestamp(Timestamp ts) noexcept { timestamp_ns_ = ts; }

  void setCenter(const Point& center) noexcept { center_ = center; }
  void setOrientation(const Quaternion& orientation) noexcept {
    orientation_ = orientation.normalized();
  }
  void setExtent(const Vector3& extent) noexcept { extent_ = extent; }

  // ========== Query Methods ==========

  bool isEmpty() const noexcept {
    return extent_.x() <= 0.0f || extent_.y() <= 0.0f || extent_.z() <= 0.0f;
  }

  float volume() const noexcept {
    if (isEmpty()) return 0.0f;
    Vector3 s = size();
    return s.x() * s.y() * s.z();
  }

  bool contains(const Point& point) const noexcept {
    Vector3 local = orientation_.conjugate() * (point - center_);
    return (local.x() >= -extent_.x() && local.x() <= extent_.x() &&
            local.y() >= -extent_.y() && local.y() <= extent_.y() &&
            local.z() >= -extent_.z() && local.z() <= extent_.z());
  }

  AxisAlignedBox toAABB() const {
    auto corners_array = corners();
    Point min_pt = Point::Constant(std::numeric_limits<float>::max());
    Point max_pt = Point::Constant(std::numeric_limits<float>::lowest());

    for (const auto& corner : corners_array) {
      min_pt = min_pt.cwiseMin(corner);
      max_pt = max_pt.cwiseMax(corner);
    }

    return AxisAlignedBox::fromMinMax(min_pt, max_pt, frame_id_, timestamp_ns_);
  }

  Point corner(int index) const {
    Vector3 local((index & 1) ? extent_.x() : -extent_.x(),
                  (index & 2) ? extent_.y() : -extent_.y(),
                  (index & 4) ? extent_.z() : -extent_.z());
    return center_ + orientation_ * local;
  }

  std::array<Point, 8> corners() const {
    std::array<Point, 8> result;
    for (int i = 0; i < 8; ++i) {
      result[i] = corner(i);
    }
    return result;
  }
};

using OBB = OrientedBox;

}  // namespace nanopcl

#endif  // NANOPCL_GEOMETRY_BBOX_HPP
