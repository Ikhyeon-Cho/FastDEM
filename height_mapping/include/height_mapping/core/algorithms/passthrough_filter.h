/*
 * passthrough_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ALGORITHMS_PASSTHROUGH_FILTER_H
#define HEIGHT_MAPPING_CORE_ALGORITHMS_PASSTHROUGH_FILTER_H

#include <limits>

#include "height_mapping/core/data/point_cloud.h"

namespace height_mapping::algorithms {

/**
 * @brief Passthrough filter algorithm for spatial and distance-based filtering
 *
 * Filters points based on axis-aligned bounds and optional distance
 * constraints.
 */
class PassthroughFilter {
 public:
  /**
   * @brief Configuration parameters for passthrough filter
   */
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
        throw std::invalid_argument("Min bounds must be less than max bounds");
      }
      if (use_distance_filter && distance_min > distance_max) {
        throw std::invalid_argument(
            "distance_min must be less than distance_max");
      }
    }
  };

  /**
   * @brief Statistics from filtering operation
   */
  struct Stats {
    size_t original_size = 0;
    size_t filtered_size = 0;
    size_t removed_count = 0;
  };

 private:
  Config config_;

 public:
  PassthroughFilter() : config_{} { config_.validate(); }

  explicit PassthroughFilter(const Config &config) : config_(config) {
    config_.validate();
  }

  void setConfig(const Config &config) {
    config_ = config;
    config_.validate();
  }

  const Config &getConfig() const { return config_; }

  /**
   * @brief Apply passthrough filter in-place
   * @param cloud Point cloud to filter (modified in-place)
   * @return Statistics from the filtering operation
   */
  Stats filterInPlace(PointCloud &cloud) const;

  /**
   * @brief Apply passthrough filter to copy of cloud
   * @param cloud Input point cloud (not modified)
   * @param stats Statistics from the filtering operation
   * @return Filtered copy of the cloud
   */
  PointCloud filter(const PointCloud &cloud, Stats &stats) const;

 private:
  bool passesFilter(PointCloud::PointView point) const;
};

}  // namespace height_mapping::algorithms

#endif  // HEIGHT_MAPPING_CORE_ALGORITHMS_PASSTHROUGH_FILTER_H