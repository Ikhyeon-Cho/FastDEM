// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_IMPL_POINT_CLOUD_V3_IMPL_HPP
#define NANOPCL_CORE_IMPL_POINT_CLOUD_V3_IMPL_HPP

#include "nanopcl/core/point_cloud_v3.hpp"

namespace nanopcl {

// Size & Capacity

inline void PointCloud::resize(size_t n) {
  points_.resize(n, Point(0, 0, 0, 1));
  syncChannelSizes(n);
}

inline void PointCloud::reserve(size_t n) {
  points_.reserve(n);
  if (hasIntensity()) intensity_.reserve(n);
  if (hasTime()) time_.reserve(n);
  if (hasRing()) ring_.reserve(n);
  if (hasColor()) color_.reserve(n);
  if (hasLabel()) label_.reserve(n);
  if (hasNormal()) normal_.reserve(n);
  if (hasCovariance()) covariance_.reserve(n);
}

inline void PointCloud::clear() {
  // Clear data but keep channel structure
  points_.clear();
  if (hasIntensity()) intensity_.clear();
  if (hasTime()) time_.clear();
  if (hasRing()) ring_.clear();
  if (hasColor()) color_.clear();
  if (hasLabel()) label_.clear();
  if (hasNormal()) normal_.clear();
  if (hasCovariance()) covariance_.clear();
}

inline void PointCloud::reset() {
  // Full reset - clear everything including channels
  points_.clear();
  points_.shrink_to_fit();

  intensity_.clear();
  intensity_.shrink_to_fit();
  time_.clear();
  time_.shrink_to_fit();
  ring_.clear();
  ring_.shrink_to_fit();
  color_.clear();
  color_.shrink_to_fit();
  label_.clear();
  label_.shrink_to_fit();
  normal_.clear();
  normal_.shrink_to_fit();
  covariance_.clear();
  covariance_.shrink_to_fit();

  // Reset channel flags
  use_intensity_ = false;
  use_time_ = false;
  use_ring_ = false;
  use_color_ = false;
  use_label_ = false;
  use_normal_ = false;
  use_covariance_ = false;

  frame_id_.clear();
  timestamp_ns_ = 0;
}

// Channel Enable

inline void PointCloud::enableIntensity() {
  use_intensity_ = true;
  if (intensity_.empty() && !points_.empty()) {
    intensity_.resize(points_.size(), 0.0f);
  } else if (intensity_.empty()) {
    intensity_.reserve(points_.capacity());
  }
}

inline void PointCloud::enableTime() {
  use_time_ = true;
  if (time_.empty() && !points_.empty()) {
    time_.resize(points_.size(), 0.0f);
  } else if (time_.empty()) {
    time_.reserve(points_.capacity());
  }
}

inline void PointCloud::enableRing() {
  use_ring_ = true;
  if (ring_.empty() && !points_.empty()) {
    ring_.resize(points_.size(), 0);
  } else if (ring_.empty()) {
    ring_.reserve(points_.capacity());
  }
}

inline void PointCloud::enableColor() {
  use_color_ = true;
  if (color_.empty() && !points_.empty()) {
    color_.resize(points_.size(), Color());
  } else if (color_.empty()) {
    color_.reserve(points_.capacity());
  }
}

inline void PointCloud::enableLabel() {
  use_label_ = true;
  if (label_.empty() && !points_.empty()) {
    label_.resize(points_.size(), Label());
  } else if (label_.empty()) {
    label_.reserve(points_.capacity());
  }
}

inline void PointCloud::enableNormal() {
  use_normal_ = true;
  if (normal_.empty() && !points_.empty()) {
    normal_.resize(points_.size(), Normal(0, 0, 1, 0));
  } else if (normal_.empty()) {
    normal_.reserve(points_.capacity());
  }
}

inline void PointCloud::enableCovariance() {
  use_covariance_ = true;
  if (covariance_.empty() && !points_.empty()) {
    covariance_.resize(points_.size(), Eigen::Matrix3f::Identity());
  } else if (covariance_.empty()) {
    covariance_.reserve(points_.capacity());
  }
}

// Point Addition

inline void PointCloud::add(float x, float y, float z) {
  points_.push_back(Point(x, y, z, 1));
  pushDefaultChannelValues();
}

inline void PointCloud::add(float x, float y, float z, Intensity intensity) {
  if (!hasIntensity()) enableIntensity();
  points_.push_back(Point(x, y, z, 1));
  pushDefaultChannelValues();
  intensity_.back() = intensity;
}

inline void PointCloud::add(float x, float y, float z, Intensity intensity, Ring ring) {
  if (!hasIntensity()) enableIntensity();
  if (!hasRing()) enableRing();
  points_.push_back(Point(x, y, z, 1));
  pushDefaultChannelValues();
  intensity_.back() = intensity;
  ring_.back() = ring;
}

inline void PointCloud::add(float x, float y, float z, Intensity intensity, Ring ring, Time time) {
  if (!hasIntensity()) enableIntensity();
  if (!hasRing()) enableRing();
  if (!hasTime()) enableTime();
  points_.push_back(Point(x, y, z, 1));
  pushDefaultChannelValues();
  intensity_.back() = intensity;
  ring_.back() = ring;
  time_.back() = time;
}

inline void PointCloud::add(const Point& p) {
  Point pt = p;
  pt.w() = 1.0f;  // Ensure w=1
  points_.push_back(pt);
  pushDefaultChannelValues();
}

// Channel Utilities

inline void PointCloud::copyChannelLayout(const PointCloud& other) {
  if (other.hasIntensity()) enableIntensity();
  if (other.hasTime()) enableTime();
  if (other.hasRing()) enableRing();
  if (other.hasColor()) enableColor();
  if (other.hasLabel()) enableLabel();
  if (other.hasNormal()) enableNormal();
  if (other.hasCovariance()) enableCovariance();
}

inline void PointCloud::copyChannelData(size_t dst_idx, const PointCloud& src, size_t src_idx) {
  if (hasIntensity() && src.hasIntensity()) intensity_[dst_idx] = src.intensity_[src_idx];
  if (hasTime() && src.hasTime()) time_[dst_idx] = src.time_[src_idx];
  if (hasRing() && src.hasRing()) ring_[dst_idx] = src.ring_[src_idx];
  if (hasColor() && src.hasColor()) color_[dst_idx] = src.color_[src_idx];
  if (hasLabel() && src.hasLabel()) label_[dst_idx] = src.label_[src_idx];
  if (hasNormal() && src.hasNormal()) normal_[dst_idx] = src.normal_[src_idx];
  if (hasCovariance() && src.hasCovariance()) covariance_[dst_idx] = src.covariance_[src_idx];
}

inline void PointCloud::appendChannelData(const PointCloud& src) {
  if (hasIntensity() && src.hasIntensity())
    intensity_.insert(intensity_.end(), src.intensity_.begin(), src.intensity_.end());
  if (hasTime() && src.hasTime())
    time_.insert(time_.end(), src.time_.begin(), src.time_.end());
  if (hasRing() && src.hasRing())
    ring_.insert(ring_.end(), src.ring_.begin(), src.ring_.end());
  if (hasColor() && src.hasColor())
    color_.insert(color_.end(), src.color_.begin(), src.color_.end());
  if (hasLabel() && src.hasLabel())
    label_.insert(label_.end(), src.label_.begin(), src.label_.end());
  if (hasNormal() && src.hasNormal())
    normal_.insert(normal_.end(), src.normal_.begin(), src.normal_.end());
  if (hasCovariance() && src.hasCovariance())
    covariance_.insert(covariance_.end(), src.covariance_.begin(), src.covariance_.end());
}

// Operations

inline PointCloud& PointCloud::operator+=(const PointCloud& other) {
  if (other.empty()) return *this;

  // Copy layout if this cloud is empty
  if (empty()) {
    copyChannelLayout(other);
  }

  // Reserve space
  size_t new_size = size() + other.size();
  reserve(new_size);

  // Append points
  points_.insert(points_.end(), other.points_.begin(), other.points_.end());

  // Append channel data
  appendChannelData(other);

  // Sync sizes for channels that exist in this but not in other
  syncChannelSizes(new_size);

  return *this;
}

inline PointCloud PointCloud::extract(const std::vector<size_t>& indices) const {
  PointCloud result;
  result.copyChannelLayout(*this);
  result.reserve(indices.size());

  for (size_t idx : indices) {
    result.points_.push_back(points_[idx]);
    if (hasIntensity()) result.intensity_.push_back(intensity_[idx]);
    if (hasTime()) result.time_.push_back(time_[idx]);
    if (hasRing()) result.ring_.push_back(ring_[idx]);
    if (hasColor()) result.color_.push_back(color_[idx]);
    if (hasLabel()) result.label_.push_back(label_[idx]);
    if (hasNormal()) result.normal_.push_back(normal_[idx]);
    if (hasCovariance()) result.covariance_.push_back(covariance_[idx]);
  }

  result.frame_id_ = frame_id_;
  result.timestamp_ns_ = timestamp_ns_;
  return result;
}

inline void PointCloud::erase(const std::vector<size_t>& indices) {
  if (indices.empty()) return;

  // Create sorted unique indices
  std::vector<size_t> sorted_indices = indices;
  std::sort(sorted_indices.begin(), sorted_indices.end());
  sorted_indices.erase(std::unique(sorted_indices.begin(), sorted_indices.end()),
                       sorted_indices.end());

  // Compact: copy valid elements to new positions
  size_t write = 0;
  size_t idx_pos = 0;
  for (size_t read = 0; read < points_.size(); ++read) {
    if (idx_pos < sorted_indices.size() && read == sorted_indices[idx_pos]) {
      ++idx_pos;  // Skip this index
      continue;
    }
    if (write != read) {
      points_[write] = points_[read];
      if (hasIntensity()) intensity_[write] = intensity_[read];
      if (hasTime()) time_[write] = time_[read];
      if (hasRing()) ring_[write] = ring_[read];
      if (hasColor()) color_[write] = color_[read];
      if (hasLabel()) label_[write] = label_[read];
      if (hasNormal()) normal_[write] = normal_[read];
      if (hasCovariance()) covariance_[write] = covariance_[read];
    }
    ++write;
  }

  resize(write);
}

// Internal Helpers

inline void PointCloud::pushDefaultChannelValues() {
  if (hasIntensity()) intensity_.push_back(0.0f);
  if (hasTime()) time_.push_back(0.0f);
  if (hasRing()) ring_.push_back(0);
  if (hasColor()) color_.push_back(Color());
  if (hasLabel()) label_.push_back(Label());
  if (hasNormal()) normal_.push_back(Normal(0, 0, 1, 0));
  if (hasCovariance()) covariance_.push_back(Eigen::Matrix3f::Identity());
}

inline void PointCloud::syncChannelSizes(size_t n) {
  if (hasIntensity()) intensity_.resize(n, 0.0f);
  if (hasTime()) time_.resize(n, 0.0f);
  if (hasRing()) ring_.resize(n, 0);
  if (hasColor()) color_.resize(n, Color());
  if (hasLabel()) label_.resize(n, Label());
  if (hasNormal()) normal_.resize(n, Normal(0, 0, 1, 0));
  if (hasCovariance()) covariance_.resize(n, Eigen::Matrix3f::Identity());
}

}  // namespace nanopcl

#endif  // NANOPCL_CORE_IMPL_POINT_CLOUD_V3_IMPL_HPP
