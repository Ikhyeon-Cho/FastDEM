/*
 * statistical_ground_segmentation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ALGORITHMS_STATISTICAL_GROUND_SEGMENTATION_H
#define HEIGHT_MAPPING_CORE_ALGORITHMS_STATISTICAL_GROUND_SEGMENTATION_H

#include "height_mapping_core/data/point_cloud.h"
#include <vector>

namespace height_mapping::algorithms {

/**
 * @brief Statistical ground segmentation algorithm
 *
 * Segments point cloud into ground and non-ground points using statistical
 * analysis. Uses percentile-based robust minimum to handle noise and outliers.
 */
class StatisticalGroundSegmentation {
public:
  /**
   * @brief Configuration parameters for statistical segmentation
   */
  struct Config {
    float ground_percentile = 0.1f; // Percentile for robust minimum (0-1)
    float ground_thickness = 0.3f;  // Thickness of ground layer in meters
    float noise_threshold = 0.5f;   // Threshold below percentile for noise
    bool keep_only_ground = true;   // Filter output to ground points only

    /**
     * @brief Validate configuration parameters
     * @throws std::invalid_argument if parameters are invalid
     */
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

  /**
   * @brief Statistics from segmentation operation
   */
  struct Stats {
    size_t ground_count = 0;
    size_t obstacle_count = 0;
    size_t noise_count = 0;
    float robust_min_z = 0;
    float ground_threshold = 0;
  };

private:
  Config config_;

public:
  /**
   * @brief Default constructor with default configuration
   */
  StatisticalGroundSegmentation() : config_{} { config_.validate(); }

  /**
   * @brief Constructor with configuration
   * @param config Configuration parameters
   */
  explicit StatisticalGroundSegmentation(const Config &config)
      : config_(config) {
    config_.validate();
  }

  /**
   * @brief Set new configuration
   * @param config New configuration parameters
   */
  void setConfig(const Config &config) {
    config_ = config;
    config_.validate();
  }

  /**
   * @brief Get current configuration
   * @return Current configuration
   */
  const Config &getConfig() const { return config_; }

  /**
   * @brief Segment point cloud in-place
   * @param cloud Point cloud to segment (modified in-place)
   * @return Statistics from the segmentation operation
   */
  Stats segmentInPlace(PointCloud &cloud) const;

  /**
   * @brief Segment copy of point cloud
   * @param cloud Input point cloud (not modified)
   * @param stats Statistics from the segmentation operation
   * @return Segmented copy of the cloud
   */
  PointCloud segment(const PointCloud &cloud, Stats &stats) const;

private:
  /**
   * @brief Calculate robust minimum Z value using percentile
   * @param z_values Sorted Z values
   * @return Robust minimum Z value
   */
  float calculateRobustMinimum(const std::vector<float> &z_values) const;

  /**
   * @brief Classify and filter points based on thresholds
   * @param cloud Input cloud
   * @param robust_min_z Robust minimum Z value
   * @param ground_threshold Ground threshold value
   * @param stats Statistics to fill
   * @return Filtered cloud
   */
  PointCloud classifyAndFilter(const PointCloud &cloud, float robust_min_z,
                               float ground_threshold, Stats &stats) const;
};

} // namespace height_mapping::algorithms

#endif // HEIGHT_MAPPING_CORE_ALGORITHMS_STATISTICAL_GROUND_SEGMENTATION_H