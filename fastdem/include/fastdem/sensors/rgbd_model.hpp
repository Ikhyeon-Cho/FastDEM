// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * rgbd_model.hpp
 *
 * RGB-D (structured light) sensor noise model for height mapping.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_SENSORS_RGBD_MODEL_HPP
#define FASTDEM_SENSORS_RGBD_MODEL_HPP

#include "fastdem/sensors/sensor_model.hpp"

namespace fastdem {

/**
 * @brief RGB-D (structured light) sensor model.
 *
 * Axis-aligned noise model based on Nguyen et al. (2012):
 *   - Normal noise (σ_norm): along sensor Z axis (depth direction)
 *   - Lateral noise (σ_lat): in sensor XY plane
 *
 * Depth noise model (quadratic):
 *   σ_norm = a + b × (d - c)²
 *   where d is depth (sensor Z), c is optimal depth
 *
 * Lateral noise model (linear):
 *   σ_lat = lateral_factor × d
 *
 * Sensor-frame covariance (diagonal):
 *   Σ_sensor = diag(σ_lat², σ_lat², σ_norm²)
 *
 * Reference:
 *   Nguyen, C. V., Izadi, S., & Lovell, D. (2012).
 *   "Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking"
 *
 * Typical values (Realsense D435):
 *   - a: 0.000611
 *   - b: 0.003587
 *   - c: 0.3515
 *   - lateral_factor: 0.01576
 *   - min_depth: 0.2m, max_depth: 3.25m
 */
class RGBDSensorModel : public SensorModel {
 public:
  /**
   * @brief Construct with RGB-D noise parameters.
   *
   * @param normal_a Base depth noise [m]
   * @param normal_b Quadratic depth noise coefficient [m⁻¹]
   * @param normal_c Optimal depth (minimum noise) [m]
   * @param lateral_factor Lateral noise factor (σ_lat = factor × depth)
   */
  RGBDSensorModel(float normal_a = 0.001f, float normal_b = 0.002f,
                  float normal_c = 0.4f, float lateral_factor = 0.001f);

  Eigen::Matrix3f computeCovariance(
      const Eigen::Vector3f& point_sensor) const override;

 private:
  static constexpr float fallback_variance_ = 0.01f;  // [m²]

  float normal_a_;        ///< Base depth noise [m]
  float normal_b_;        ///< Quadratic coefficient [m⁻¹]
  float normal_c_;        ///< Optimal depth [m]
  float lateral_factor_;  ///< Lateral noise factor
};

inline RGBDSensorModel::RGBDSensorModel(float normal_a, float normal_b,
                                        float normal_c, float lateral_factor)
    : normal_a_(normal_a),
      normal_b_(normal_b),
      normal_c_(normal_c),
      lateral_factor_(lateral_factor) {}

inline Eigen::Matrix3f RGBDSensorModel::computeCovariance(
    const Eigen::Vector3f& point_sensor) const {
  const float depth = point_sensor.z();

  if (depth <= 0.0f) {
    return Eigen::Matrix3f::Identity() * fallback_variance_;
  }

  // Depth noise: σ_norm = a + b × (d - c)²
  const float diff = depth - normal_c_;
  const float sigma_norm = normal_a_ + normal_b_ * diff * diff;
  const float var_norm = sigma_norm * sigma_norm;

  // Lateral noise: σ_lat = lateral_factor × d
  const float sigma_lat = lateral_factor_ * depth;
  const float var_lat = sigma_lat * sigma_lat;

  // Diagonal covariance: diag(σ_lat², σ_lat², σ_norm²)
  return Eigen::Vector3f(var_lat, var_lat, var_norm).asDiagonal();
}

}  // namespace fastdem

#endif  // FASTDEM_SENSORS_RGBD_MODEL_HPP
