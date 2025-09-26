/*
 * raycasting.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ALGORITHMS_RAYCASTING_H
#define HEIGHT_MAPPING_CORE_ALGORITHMS_RAYCASTING_H

#include "height_mapping_core/data/height_map.h"
#include "height_mapping_core/data/point_cloud.h"
#include <Eigen/Core>
#include <cmath>

namespace height_mapping::algorithms {

/**
 * @brief Raycasting algorithm for height map correction
 *
 * Traces rays from sensor to points to detect and correct false ground heights.
 * Ensures ground elevation is not higher than ray paths.
 */
class Raycasting {
public:
  /**
   * @brief Configuration parameters for raycasting
   */
  struct Config {
    float max_ground_angle = 45.0f * M_PI / 180.0f; // radians
    float correction_threshold = 0.1f;              // meters
    bool enable_correction = true;
    float ray_step_size = 0.0f; // 0 = use map resolution

    void validate() const {
      if (max_ground_angle < 0 || max_ground_angle > M_PI / 2) {
        throw std::invalid_argument(
            "max_ground_angle must be in [0, 90] degrees");
      }
      if (correction_threshold < 0) {
        throw std::invalid_argument("correction_threshold must be positive");
      }
      if (ray_step_size < 0) {
        throw std::invalid_argument("ray_step_size must be non-negative");
      }
    }
  };

  /**
   * @brief Raycasting statistics
   */
  struct Stats {
    size_t total_rays_cast = 0;
    size_t ground_points_detected = 0;
    size_t cells_corrected = 0;
    size_t cells_traced = 0;
  };

private:
  Config config_;
  mutable Stats cumulative_stats_; // Track cumulative statistics

public:
  Raycasting() : config_{} { config_.validate(); }

  explicit Raycasting(const Config &config) : config_(config) {
    config_.validate();
  }

  void setConfig(const Config &config) {
    config_ = config;
    config_.validate();
  }

  const Config &getConfig() const { return config_; }
  const Stats &getCumulativeStats() const { return cumulative_stats_; }
  void resetStats() { cumulative_stats_ = Stats(); }

  /**
   * @brief Apply raycasting correction to height map
   * @param map Height map to correct (modified)
   * @param cloud Point cloud with sensor measurements
   * @param sensor_origin Sensor position in map frame
   * @return Raycasting statistics for this operation
   */
  Stats correctMap(HeightMap &map, const PointCloud &cloud,
                   const Eigen::Vector3f &sensor_origin) const;

private:
  void traceRays(HeightMap &map, const PointCloud &cloud,
                 const Eigen::Vector3f &sensor_origin, Stats &stats) const;

  void applyCorrections(HeightMap &map, Stats &stats) const;

  bool isGroundRay(const PointCloud::ConstPointView &point,
                   const Eigen::Vector3f &sensor_origin) const;

  float computeRayAngle(const PointCloud::ConstPointView &point,
                        const Eigen::Vector3f &sensor_origin) const;
};

} // namespace height_mapping::algorithms

#endif // HEIGHT_MAPPING_CORE_ALGORITHMS_RAYCASTING_H