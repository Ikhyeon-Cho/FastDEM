/*
 * configs.h
 *
 * Configuration structures for height mapping algorithms.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ALGORITHMS_CONFIGS_H
#define HEIGHT_MAPPING_ALGORITHMS_CONFIGS_H

#include <stdexcept>
#include <string>

namespace height_mapping::algorithms {

// =============================================================================
// Raycasting
// =============================================================================

struct RaycastingConfig {
  bool enable_correction = true;
  float max_ground_angle = -5.0f * 3.14159265f / 180.0f;  // [rad]
  float correction_threshold = 0.05f;  // [m]
  float ray_step_size = 0.1f;          // [m]

  void validate() const {
    if (correction_threshold <= 0) {
      throw std::invalid_argument("correction_threshold must be positive");
    }
    if (ray_step_size <= 0) {
      throw std::invalid_argument("ray_step_size must be positive");
    }
  }
};

// =============================================================================
// Grid Ground Segmentation
// =============================================================================

struct GridGroundSegmentationConfig {
  float grid_resolution = 0.5f;     // Grid cell size in meters
  float cell_percentile = 0.2f;     // Percentile for robust minimum (0-1)
  float ground_thickness = 0.3f;    // Thickness of ground layer in meters
  float max_ground_height = 0.5f;   // Maximum height to consider as ground
  size_t min_points_per_cell = 2;   // Minimum points to process a cell
  bool keep_only_ground = true;     // Filter output to ground points only

  void validate() const {
    if (grid_resolution <= 0) {
      throw std::invalid_argument("grid_resolution must be positive");
    }
    if (cell_percentile < 0 || cell_percentile > 1) {
      throw std::invalid_argument("cell_percentile must be in [0, 1]");
    }
    if (ground_thickness <= 0) {
      throw std::invalid_argument("ground_thickness must be positive");
    }
    if (max_ground_height <= 0) {
      throw std::invalid_argument("max_ground_height must be positive");
    }
  }
};

// =============================================================================
// Statistical Ground Segmentation
// =============================================================================

struct StatisticalGroundSegmentationConfig {
  float ground_percentile = 0.1f;  // Percentile for robust minimum (0-1)
  float ground_thickness = 0.3f;   // Thickness of ground layer in meters
  float noise_threshold = 0.5f;    // Threshold below percentile for noise
  bool keep_only_ground = true;    // Filter output to ground points only

  void validate() const {
    if (ground_percentile < 0 || ground_percentile > 1) {
      throw std::invalid_argument("ground_percentile must be in [0, 1]");
    }
    if (ground_thickness <= 0) {
      throw std::invalid_argument("ground_thickness must be positive");
    }
    if (noise_threshold < 0) {
      throw std::invalid_argument("noise_threshold must be non-negative");
    }
  }
};

struct GroundSegmentationStats {
  size_t ground_count = 0;
  size_t obstacle_count = 0;
  size_t noise_count = 0;
  float robust_min_z = 0;
  float ground_threshold = 0;
};

}  // namespace height_mapping::algorithms

#endif  // HEIGHT_MAPPING_ALGORITHMS_CONFIGS_H
