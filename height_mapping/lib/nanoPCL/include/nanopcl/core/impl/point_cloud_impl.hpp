// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for PointCloud.
// Do not include this file directly; include <nanopcl/core/point_cloud.hpp>

#ifndef NANOPCL_CORE_IMPL_POINT_CLOUD_IMPL_HPP
#define NANOPCL_CORE_IMPL_POINT_CLOUD_IMPL_HPP

#include <algorithm>
#include <cassert>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <utility>

namespace npcl {

namespace detail {

/// Helper for copy assignment: reuses existing buffer when possible
template <typename T>
void copyChannel(std::unique_ptr<std::vector<T>>& dest,
                 const std::unique_ptr<std::vector<T>>& src) {
  if (src) {
    if (dest) {
      *dest = *src;  // Reuse existing buffer (key optimization)
    } else {
      dest = std::make_unique<std::vector<T>>(*src);
    }
  } else {
    dest.reset();
  }
}

}  // namespace detail

// ============================================================================
// Constructors
// ============================================================================

inline PointCloud::PointCloud() { points_.reserve(DEFAULT_CAPACITY); }

inline PointCloud::PointCloud(size_t reserve_size) {
  points_.reserve(reserve_size);
}

inline PointCloud::PointCloud(const std::string& frame_id, size_t reserve_size)
    : frame_id_(frame_id) {
  points_.reserve(reserve_size);
}

// ============================================================================
// Rule of 5 (Copy/Move)
// ============================================================================

inline PointCloud::PointCloud(const PointCloud& other)
    : points_(other.points_),
      frame_id_(other.frame_id_),
      timestamp_ns_(other.timestamp_ns_) {
  if (other.intensity_) {
    intensity_ = std::make_unique<std::vector<float>>(*other.intensity_);
  }
  if (other.time_) {
    time_ = std::make_unique<std::vector<float>>(*other.time_);
  }
  if (other.ring_) {
    ring_ = std::make_unique<std::vector<uint16_t>>(*other.ring_);
  }
  if (other.color_) {
    color_ = std::make_unique<std::vector<Color>>(*other.color_);
  }
  if (other.label_) {
    label_ = std::make_unique<std::vector<Label>>(*other.label_);
  }
  if (other.normal_) {
    normal_ = std::make_unique<std::vector<Eigen::Vector3f>>(*other.normal_);
  }
  if (other.covariance_) {
    covariance_ =
        std::make_unique<std::vector<Eigen::Matrix3f>>(*other.covariance_);
  }
}

inline PointCloud& PointCloud::operator=(const PointCloud& other) {
  if (this == &other) return *this;

  points_ = other.points_;
  frame_id_ = other.frame_id_;
  timestamp_ns_ = other.timestamp_ns_;

  detail::copyChannel(intensity_, other.intensity_);
  detail::copyChannel(time_, other.time_);
  detail::copyChannel(ring_, other.ring_);
  detail::copyChannel(color_, other.color_);
  detail::copyChannel(label_, other.label_);
  detail::copyChannel(normal_, other.normal_);
  detail::copyChannel(covariance_, other.covariance_);

  return *this;
}

// ============================================================================
// Merge Operator
// ============================================================================

inline PointCloud& PointCloud::operator+=(const PointCloud& other) {
  if (other.empty()) return *this;

  // Frame compatibility check
  if (!frame_id_.empty() && !other.frame_id_.empty() &&
      frame_id_ != other.frame_id_) {
    throw std::runtime_error(
        "Cannot merge point clouds with different frames: '" + frame_id_ +
        "' vs '" + other.frame_id_ + "'");
  }

  // Propagate frame if this is empty
  if (frame_id_.empty()) {
    frame_id_ = other.frame_id_;
  }

  // Pre-allocate to avoid multiple reallocations
  const size_t new_size = size() + other.size();
  points_.reserve(new_size);
  if (intensity_) intensity_->reserve(new_size);
  if (time_) time_->reserve(new_size);
  if (ring_) ring_->reserve(new_size);
  if (color_) color_->reserve(new_size);
  if (label_) label_->reserve(new_size);
  if (normal_) normal_->reserve(new_size);
  if (covariance_) covariance_->reserve(new_size);

  points_.insert(points_.end(), other.points_.begin(), other.points_.end());

  if (intensity_) {
    if (other.intensity_) {
      intensity_->insert(intensity_->end(), other.intensity_->begin(),
                         other.intensity_->end());
    } else {
      intensity_->resize(points_.size(), 0.0f);
    }
  }

  if (time_) {
    if (other.time_) {
      time_->insert(time_->end(), other.time_->begin(), other.time_->end());
    } else {
      time_->resize(points_.size(), 0.0f);
    }
  }

  if (ring_) {
    if (other.ring_) {
      ring_->insert(ring_->end(), other.ring_->begin(), other.ring_->end());
    } else {
      ring_->resize(points_.size(), 0);
    }
  }

  if (color_) {
    if (other.color_) {
      color_->insert(color_->end(), other.color_->begin(), other.color_->end());
    } else {
      color_->resize(points_.size(), Color());
    }
  }

  if (label_) {
    if (other.label_) {
      label_->insert(label_->end(), other.label_->begin(), other.label_->end());
    } else {
      label_->resize(points_.size(), Label());
    }
  }

  if (normal_) {
    if (other.normal_) {
      normal_->insert(normal_->end(), other.normal_->begin(),
                      other.normal_->end());
    } else {
      normal_->resize(points_.size(), Eigen::Vector3f::Zero());
    }
  }

  if (covariance_) {
    if (other.covariance_) {
      covariance_->insert(covariance_->end(), other.covariance_->begin(),
                          other.covariance_->end());
    } else {
      covariance_->resize(points_.size(), Eigen::Matrix3f::Zero());
    }
  }

  return *this;
}

inline PointCloud PointCloud::operator+(const PointCloud& other) const {
  PointCloud result(*this);
  result += other;
  return result;
}

// ============================================================================
// Intensity Channel
// ============================================================================

inline void PointCloud::enableIntensity() {
  if (!intensity_) {
    intensity_ = std::make_unique<std::vector<float>>();
    intensity_->reserve(points_.capacity());
    intensity_->resize(points_.size(), 0.0f);
  }
}

inline std::vector<float>& PointCloud::intensity() {
  if (!intensity_) {
    throw std::runtime_error(
        "Intensity channel not enabled. Call enableIntensity() first.");
  }
  return *intensity_;
}

inline const std::vector<float>& PointCloud::intensity() const {
  if (!intensity_) {
    throw std::runtime_error(
        "Intensity channel not enabled. Call enableIntensity() first.");
  }
  return *intensity_;
}

// ============================================================================
// Time Channel
// ============================================================================

inline void PointCloud::enableTime() {
  if (!time_) {
    time_ = std::make_unique<std::vector<float>>();
    time_->reserve(points_.capacity());
    time_->resize(points_.size(), 0.0f);
  }
}

inline std::vector<float>& PointCloud::time() {
  if (!time_) {
    throw std::runtime_error(
        "Time channel not enabled. Call enableTime() first.");
  }
  return *time_;
}

inline const std::vector<float>& PointCloud::time() const {
  if (!time_) {
    throw std::runtime_error(
        "Time channel not enabled. Call enableTime() first.");
  }
  return *time_;
}

// ============================================================================
// Ring Channel
// ============================================================================

inline void PointCloud::enableRing() {
  if (!ring_) {
    ring_ = std::make_unique<std::vector<uint16_t>>();
    ring_->reserve(points_.capacity());
    ring_->resize(points_.size(), 0);
  }
}

inline std::vector<uint16_t>& PointCloud::ring() {
  if (!ring_) {
    throw std::runtime_error(
        "Ring channel not enabled. Call enableRing() first.");
  }
  return *ring_;
}

inline const std::vector<uint16_t>& PointCloud::ring() const {
  if (!ring_) {
    throw std::runtime_error(
        "Ring channel not enabled. Call enableRing() first.");
  }
  return *ring_;
}

// ============================================================================
// Color Channel
// ============================================================================

inline void PointCloud::enableColor() {
  if (!color_) {
    color_ = std::make_unique<std::vector<Color>>();
    color_->reserve(points_.capacity());
    color_->resize(points_.size(), Color());
  }
}

inline std::vector<Color>& PointCloud::color() {
  if (!color_) {
    throw std::runtime_error(
        "Color channel not enabled. Call enableColor() first.");
  }
  return *color_;
}

inline const std::vector<Color>& PointCloud::color() const {
  if (!color_) {
    throw std::runtime_error(
        "Color channel not enabled. Call enableColor() first.");
  }
  return *color_;
}

// ============================================================================
// Label Channel
// ============================================================================

inline void PointCloud::enableLabel() {
  if (!label_) {
    label_ = std::make_unique<std::vector<Label>>();
    label_->reserve(points_.capacity());
    label_->resize(points_.size(), Label());
  }
}

inline std::vector<Label>& PointCloud::label() {
  if (!label_) {
    throw std::runtime_error(
        "Label channel not enabled. Call enableLabel() first.");
  }
  return *label_;
}

inline const std::vector<Label>& PointCloud::label() const {
  if (!label_) {
    throw std::runtime_error(
        "Label channel not enabled. Call enableLabel() first.");
  }
  return *label_;
}

// ============================================================================
// Normal Channel
// ============================================================================

inline void PointCloud::enableNormal() {
  if (!normal_) {
    normal_ = std::make_unique<std::vector<Eigen::Vector3f>>();
    normal_->reserve(points_.capacity());
    normal_->resize(points_.size(), Eigen::Vector3f::Zero());
  }
}

inline std::vector<Eigen::Vector3f>& PointCloud::normal() {
  if (!normal_) {
    throw std::runtime_error(
        "Normal channel not enabled. Call enableNormal() first.");
  }
  return *normal_;
}

inline const std::vector<Eigen::Vector3f>& PointCloud::normal() const {
  if (!normal_) {
    throw std::runtime_error(
        "Normal channel not enabled. Call enableNormal() first.");
  }
  return *normal_;
}

// ============================================================================
// Covariance Channel
// ============================================================================

inline void PointCloud::enableCovariance() {
  if (!covariance_) {
    covariance_ = std::make_unique<std::vector<Eigen::Matrix3f>>();
    covariance_->reserve(points_.capacity());
    covariance_->resize(points_.size(), Eigen::Matrix3f::Zero());
  }
}

inline std::vector<Eigen::Matrix3f>& PointCloud::covariance() {
  if (!covariance_) {
    throw std::runtime_error(
        "Covariance channel not enabled. Call enableCovariance() first.");
  }
  return *covariance_;
}

inline const std::vector<Eigen::Matrix3f>& PointCloud::covariance() const {
  if (!covariance_) {
    throw std::runtime_error(
        "Covariance channel not enabled. Call enableCovariance() first.");
  }
  return *covariance_;
}

// ============================================================================
// Container Operations
// ============================================================================

inline void PointCloud::reserve(size_t n) {
  points_.reserve(n);
  if (intensity_) intensity_->reserve(n);
  if (time_) time_->reserve(n);
  if (ring_) ring_->reserve(n);
  if (color_) color_->reserve(n);
  if (label_) label_->reserve(n);
  if (normal_) normal_->reserve(n);
  if (covariance_) covariance_->reserve(n);
}

inline void PointCloud::resize(size_t n) {
  points_.resize(n);
  if (intensity_) intensity_->resize(n, 0.0f);
  if (time_) time_->resize(n, 0.0f);
  if (ring_) ring_->resize(n, 0);
  if (color_) color_->resize(n, Color());
  if (label_) label_->resize(n, Label());
  if (normal_) normal_->resize(n, Eigen::Vector3f::Zero());
  if (covariance_) covariance_->resize(n, Eigen::Matrix3f::Zero());
}

inline void PointCloud::shrink_to_fit() {
  points_.shrink_to_fit();
  if (intensity_) intensity_->shrink_to_fit();
  if (time_) time_->shrink_to_fit();
  if (ring_) ring_->shrink_to_fit();
  if (color_) color_->shrink_to_fit();
  if (label_) label_->shrink_to_fit();
  if (normal_) normal_->shrink_to_fit();
  if (covariance_) covariance_->shrink_to_fit();
}

inline void PointCloud::clear() {
  points_.clear();
  if (intensity_) intensity_->clear();
  if (time_) time_->clear();
  if (ring_) ring_->clear();
  if (color_) color_->clear();
  if (label_) label_->clear();
  if (normal_) normal_->clear();
  if (covariance_) covariance_->clear();
}

inline void PointCloud::swap(PointCloud& other) noexcept {
  points_.swap(other.points_);
  frame_id_.swap(other.frame_id_);
  std::swap(timestamp_ns_, other.timestamp_ns_);
  intensity_.swap(other.intensity_);
  time_.swap(other.time_);
  ring_.swap(other.ring_);
  color_.swap(other.color_);
  label_.swap(other.label_);
  normal_.swap(other.normal_);
  covariance_.swap(other.covariance_);
}

inline void PointCloud::add(const Point& p) {
  points_.push_back(p);
  if (intensity_) intensity_->push_back(0.0f);
  if (time_) time_->push_back(0.0f);
  if (ring_) ring_->push_back(0);
  if (color_) color_->push_back(Color());
  if (label_) label_->push_back(Label());
  if (normal_) normal_->push_back(Eigen::Vector3f::Zero());
  if (covariance_) covariance_->push_back(Eigen::Matrix3f::Zero());
}

inline void PointCloud::add(Point&& p) {
  points_.push_back(std::move(p));
  if (intensity_) intensity_->push_back(0.0f);
  if (time_) time_->push_back(0.0f);
  if (ring_) ring_->push_back(0);
  if (color_) color_->push_back(Color());
  if (label_) label_->push_back(Label());
  if (normal_) normal_->push_back(Eigen::Vector3f::Zero());
  if (covariance_) covariance_->push_back(Eigen::Matrix3f::Zero());
}

// ============================================================================
// DTO Add Operations
// ============================================================================

inline void PointCloud::add(const PointXYZI& p) {
  add(Point(p.x, p.y, p.z), Intensity(p.intensity));
}

inline void PointCloud::add(const PointXYZIR& p) {
  add(Point(p.x, p.y, p.z), Intensity(p.intensity), Ring(p.ring));
}

inline void PointCloud::add(const PointXYZIT& p) {
  add(Point(p.x, p.y, p.z), Intensity(p.intensity), Time(p.time));
}

inline void PointCloud::add(const PointXYZIRT& p) {
  add(Point(p.x, p.y, p.z), Intensity(p.intensity), Ring(p.ring), Time(p.time));
}

inline void PointCloud::add(const PointXYZRGB& p) {
  add(Point(p.x, p.y, p.z), Color(p.r, p.g, p.b));
}

inline void PointCloud::add(const PointXYZRGBN& p) {
  add(Point(p.x, p.y, p.z), Color(p.r, p.g, p.b), Normal(p.nx, p.ny, p.nz));
}

inline void PointCloud::add(const PointXYZRGBL& p) {
  add(Point(p.x, p.y, p.z), Color(p.r, p.g, p.b), Label(p.label));
}

inline void PointCloud::add(const PointXYZL& p) {
  add(Point(p.x, p.y, p.z), Label(p.label));
}

inline void PointCloud::add(const PointXYZIL& p) {
  add(Point(p.x, p.y, p.z), Intensity(p.intensity), Label(p.label));
}

inline void PointCloud::add(const PointXYZN& p) {
  add(Point(p.x, p.y, p.z), Normal(p.nx, p.ny, p.nz));
}

inline void PointCloud::add(const PointXYZIN& p) {
  add(Point(p.x, p.y, p.z), Intensity(p.intensity), Normal(p.nx, p.ny, p.nz));
}

inline void PointCloud::pop_back() {
  points_.pop_back();
  if (intensity_) intensity_->pop_back();
  if (time_) time_->pop_back();
  if (ring_) ring_->pop_back();
  if (color_) color_->pop_back();
  if (label_) label_->pop_back();
  if (normal_) normal_->pop_back();
  if (covariance_) covariance_->pop_back();
}

// ============================================================================
// Erase Operations
// ============================================================================

inline PointCloud::iterator PointCloud::erase(iterator pos) {
  size_t idx = pos.index();
  if (intensity_) intensity_->erase(intensity_->begin() + idx);
  if (time_) time_->erase(time_->begin() + idx);
  if (ring_) ring_->erase(ring_->begin() + idx);
  if (color_) color_->erase(color_->begin() + idx);
  if (label_) label_->erase(label_->begin() + idx);
  if (normal_) normal_->erase(normal_->begin() + idx);
  if (covariance_) covariance_->erase(covariance_->begin() + idx);
  points_.erase(points_.begin() + idx);
  return iterator(*this, idx);
}

inline PointCloud::iterator PointCloud::erase(iterator first, iterator last) {
  size_t first_idx = first.index();
  size_t last_idx = last.index();
  if (intensity_) {
    intensity_->erase(intensity_->begin() + first_idx,
                      intensity_->begin() + last_idx);
  }
  if (time_) {
    time_->erase(time_->begin() + first_idx, time_->begin() + last_idx);
  }
  if (ring_) {
    ring_->erase(ring_->begin() + first_idx, ring_->begin() + last_idx);
  }
  if (color_) {
    color_->erase(color_->begin() + first_idx, color_->begin() + last_idx);
  }
  if (label_) {
    label_->erase(label_->begin() + first_idx, label_->begin() + last_idx);
  }
  if (normal_) {
    normal_->erase(normal_->begin() + first_idx, normal_->begin() + last_idx);
  }
  if (covariance_) {
    covariance_->erase(covariance_->begin() + first_idx,
                       covariance_->begin() + last_idx);
  }
  points_.erase(points_.begin() + first_idx, points_.begin() + last_idx);
  return iterator(*this, first_idx);
}

inline void PointCloud::erase(size_t index) {
  if (index >= size()) {
    throw std::out_of_range("PointCloud::erase: index out of range");
  }
  erase(begin() + index);
}

inline void PointCloud::erase(size_t first, size_t last) {
  if (first > last || last > size()) {
    throw std::out_of_range("PointCloud::erase: invalid range");
  }
  erase(begin() + first, begin() + last);
}

inline void PointCloud::erase(std::vector<size_t> indices) {
  if (indices.empty()) return;

  // Sort and remove duplicates
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());

  // Validate max index
  if (indices.back() >= size()) {
    throw std::out_of_range("PointCloud::erase: index out of range");
  }

  // Compact in-place: O(n) single pass
  const size_t n = size();
  const size_t num_remove = indices.size();
  size_t write = 0;
  size_t idx_ptr = 0;

  for (size_t read = 0; read < n; ++read) {
    if (idx_ptr < num_remove && read == indices[idx_ptr]) {
      ++idx_ptr;  // skip this point
    } else {
      if (write != read) {
        points_[write] = std::move(points_[read]);
        if (intensity_) (*intensity_)[write] = (*intensity_)[read];
        if (time_) (*time_)[write] = (*time_)[read];
        if (ring_) (*ring_)[write] = (*ring_)[read];
        if (color_) (*color_)[write] = (*color_)[read];
        if (label_) (*label_)[write] = (*label_)[read];
        if (normal_) (*normal_)[write] = (*normal_)[read];
        if (covariance_) (*covariance_)[write] = (*covariance_)[read];
      }
      ++write;
    }
  }

  // Truncate
  resize(write);
}

// ============================================================================
// Extract by Indices
// ============================================================================

inline PointCloud PointCloud::operator[](
    const std::vector<size_t>& indices) const {
  // Debug-only bounds checking
#ifndef NDEBUG
  for (size_t idx : indices) {
    assert(idx < size() && "PointCloud::operator[]: index out of range");
  }
#endif

  PointCloud result(frame_id_);
  result.setTimestamp(timestamp_ns_);

  const size_t n = indices.size();

  // 1. Pre-allocate all memory
  result.resize(n);
  if (intensity_) result.enableIntensity();
  if (time_) result.enableTime();
  if (ring_) result.enableRing();
  if (color_) result.enableColor();
  if (label_) result.enableLabel();
  if (normal_) result.enableNormal();
  if (covariance_) result.enableCovariance();

  // 2. Loop Fission: gather each channel separately
  //    This improves cache utilization and reduces register pressure
  {
    const Point* src = points_.data();
    Point* dst = result.points_.data();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  if (intensity_) {
    const float* src = intensity_->data();
    float* dst = result.intensity_->data();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  if (time_) {
    const float* src = time_->data();
    float* dst = result.time_->data();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  if (ring_) {
    const uint16_t* src = ring_->data();
    uint16_t* dst = result.ring_->data();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  if (color_) {
    const Color* src = color_->data();
    Color* dst = result.color_->data();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  if (label_) {
    const Label* src = label_->data();
    Label* dst = result.label_->data();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  if (normal_) {
    const Eigen::Vector3f* src = normal_->data();
    Eigen::Vector3f* dst = result.normal_->data();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  if (covariance_) {
    const Eigen::Matrix3f* src = covariance_->data();
    Eigen::Matrix3f* dst = result.covariance_->data();
    for (size_t i = 0; i < n; ++i) {
      dst[i] = src[indices[i]];
    }
  }

  return result;
}

}  // namespace npcl

#endif  // NANOPCL_CORE_IMPL_POINT_CLOUD_IMPL_HPP
