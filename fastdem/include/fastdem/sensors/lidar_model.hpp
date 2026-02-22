// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * lidar_model.hpp
 *
 * 3D LiDAR sensor model with full covariance propagation.
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_SENSORS_LIDAR_MODEL_HPP
#define FASTDEM_SENSORS_LIDAR_MODEL_HPP

#include "fastdem/sensors/sensor_model.hpp"

namespace fastdem {

/**
 * @brief 3D LiDAR sensor model with full covariance support.
 *
 * LiDAR measurements have uncertainty in two directions:
 *   - Radial (along beam): range noise σ_r
 *   - Lateral (perpendicular to beam): angular noise σ_θ × distance
 *
 * The covariance in sensor frame is computed as:
 *   Σ_sensor = σ_lat² × I + (σ_rad² - σ_lat²) × (d × d^T)
 *
 * where d is the beam direction (unit vector from sensor to point).
 *
 * Typical values:
 *   - σ_r: 0.01-0.03m (range accuracy from datasheet)
 *   - σ_θ: 0.001-0.003 rad (~0.05-0.17°, angular resolution)
 */
class LiDARSensorModel : public SensorModel {
 public:
  /**
   * @brief Construct with LiDAR noise parameters.
   *
   * @param range_noise Range uncertainty σ_r [m]
   * @param angular_noise Angular uncertainty σ_θ [rad]
   */
  LiDARSensorModel(float range_noise = 0.02f, float angular_noise = 0.001f);

  Eigen::Matrix3f computeCovariance(
      const Eigen::Vector3f& point_sensor) const override;

 private:
  static constexpr float fallback_variance_ = 0.01f;  // [m²]
  static constexpr float min_variance_ = 1e-6f;       // [m²] positive-definite floor

  float range_noise_;    ///< Range uncertainty σ_r [m]
  float angular_noise_;  ///< Angular uncertainty σ_θ [rad]
};

inline LiDARSensorModel::LiDARSensorModel(float range_noise,
                                          float angular_noise)
    : range_noise_(std::abs(range_noise)),
      angular_noise_(std::abs(angular_noise)) {}

inline Eigen::Matrix3f LiDARSensorModel::computeCovariance(
    const Eigen::Vector3f& point_sensor) const {
  const float dist_sq = point_sensor.squaredNorm();

  if (dist_sq < 1e-6f) {
    return Eigen::Matrix3f::Identity() * fallback_variance_;
  }

  const float distance = std::sqrt(dist_sq);
  const Eigen::Vector3f beam_dir = point_sensor / distance;

  // Radial variance (along beam): σ_r²
  // Lateral variance (perpendicular to beam): (d × σ_θ)²
  // Clamped to min_variance to guarantee positive-definite covariance
  const float var_radial =
      std::max(range_noise_ * range_noise_, min_variance_);
  const float var_lateral = std::max(
      (distance * angular_noise_) * (distance * angular_noise_), min_variance_);

  // Full covariance: Σ = σ_lat² × I + (σ_rad² - σ_lat²) × (d × d^T)
  // Eigenvalues: var_radial (beam), var_lateral (perpendicular) — both ≥ ε
  Eigen::Matrix3f cov = var_lateral * Eigen::Matrix3f::Identity();
  cov += (var_radial - var_lateral) * (beam_dir * beam_dir.transpose());

  return cov;
}

}  // namespace fastdem

#endif  // FASTDEM_SENSORS_LIDAR_MODEL_HPP
