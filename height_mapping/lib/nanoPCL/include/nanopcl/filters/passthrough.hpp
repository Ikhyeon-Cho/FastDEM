// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_PASSTHROUGH_HPP
#define NANOPCL_FILTERS_PASSTHROUGH_HPP

#include <cmath>
#include <limits>

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {
namespace filters {

/**
 * @brief Passthrough filter for spatial bounds filtering
 *
 * Filters points based on axis-aligned bounds and optional distance
 * constraints.
 *
 * Usage:
 * @code
 * Passthrough filter({.z_min = -1.0f, .z_max = 2.0f});
 * filter.filterInPlace(cloud);
 * @endcode
 */
class Passthrough {
 public:
  struct Config {
    // Axis-aligned bounds
    float x_min = -std::numeric_limits<float>::max();
    float x_max = std::numeric_limits<float>::max();
    float y_min = -std::numeric_limits<float>::max();
    float y_max = std::numeric_limits<float>::max();
    float z_min = -std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::max();

    // Distance bounds (from origin)
    float distance_min = 0.0f;
    float distance_max = std::numeric_limits<float>::max();
    bool use_distance_filter = false;

    void validate() const {
      if (x_min > x_max || y_min > y_max || z_min > z_max) {
        throw std::invalid_argument("Min bounds must be <= max bounds");
      }
      if (use_distance_filter && distance_min > distance_max) {
        throw std::invalid_argument("distance_min must be <= distance_max");
      }
    }
  };

  struct Stats {
    size_t input_size = 0;
    size_t output_size = 0;
    size_t removed_count = 0;
  };

 private:
  Config config_;

 public:
  Passthrough() { config_.validate(); }
  explicit Passthrough(const Config& config) : config_(config) {
    config_.validate();
  }

  void setConfig(const Config& config) {
    config_ = config;
    config_.validate();
  }

  const Config& config() const { return config_; }

  /**
   * @brief Apply passthrough filter in-place
   */
  Stats filterInPlace(PointCloud& cloud) const {
    Stats stats;
    stats.input_size = cloud.size();

    if (cloud.empty()) {
      return stats;
    }

    // Cache channel presence
    const bool has_intensity = cloud.hasIntensity();
    const bool has_time = cloud.hasTime();
    const bool has_ring = cloud.hasRing();
    const bool has_color = cloud.hasColor();
    const bool has_label = cloud.hasLabel();

    PointCloud filtered;
    filtered.reserve(cloud.size());

    // Enable channels in output
    if (has_intensity) filtered.enableIntensity();
    if (has_time) filtered.enableTime();
    if (has_ring) filtered.enableRing();
    if (has_color) filtered.enableColor();
    if (has_label) filtered.enableLabel();

    for (size_t i = 0; i < cloud.points.size(); ++i) {
      const auto& point = cloud.points[i];
      if (passes(point)) {
        filtered.points.push_back(point);
        if (has_intensity) filtered.intensity().push_back(cloud.intensity()[i]);
        if (has_time) filtered.time().push_back(cloud.time()[i]);
        if (has_ring) filtered.ring().push_back(cloud.ring()[i]);
        if (has_color) filtered.color().push_back(cloud.color()[i]);
        if (has_label) filtered.label().push_back(cloud.label()[i]);
      }
    }

    stats.output_size = filtered.size();
    stats.removed_count = stats.input_size - stats.output_size;

    // Preserve metadata
    filtered.setTimestamp(cloud.timestamp());
    filtered.setFrameId(cloud.frameId());

    cloud = std::move(filtered);
    return stats;
  }

  /**
   * @brief Apply passthrough filter to copy
   */
  PointCloud filter(const PointCloud& cloud) const {
    PointCloud result = cloud;
    filterInPlace(result);
    return result;
  }

 private:
  bool passes(const Point& p) const {
    // Check if point is finite
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()) ||
        !std::isfinite(p.z())) {
      return false;
    }

    // Check axis-aligned bounds
    if (p.x() < config_.x_min || p.x() > config_.x_max ||
        p.y() < config_.y_min || p.y() > config_.y_max ||
        p.z() < config_.z_min || p.z() > config_.z_max) {
      return false;
    }

    // Check distance filter if enabled
    if (config_.use_distance_filter) {
      float distance = p.norm();
      if (distance < config_.distance_min || distance > config_.distance_max) {
        return false;
      }
    }

    return true;
  }
};

}  // namespace filters
}  // namespace nanopcl

#endif  // NANOPCL_FILTERS_PASSTHROUGH_HPP
