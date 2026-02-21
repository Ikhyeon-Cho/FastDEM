// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_CONFIG_SENSOR_MODEL_HPP
#define FASTDEM_CONFIG_SENSOR_MODEL_HPP

namespace fastdem {

/// Sensor uncertainty model type.
enum class SensorType {
  Constant,  ///< Isotropic constant uncertainty
  LiDAR,     ///< Range-dependent model
  RGBD       ///< Depth-dependent model (Nguyen et al. 2012)
};

namespace config {

/// Sensor model parameters
/// all models share the struct, only relevant fields used
struct SensorModel {
  SensorType type = SensorType::LiDAR;

  // LiDAR
  float range_noise = 0.02f;     ///< Range uncertainty σ_r [m]
  float angular_noise = 0.001f;  ///< Angular uncertainty σ_θ [rad]

  // RGB-D (Nguyen et al. 2012)
  float normal_a = 0.001f;        ///< Base depth noise [m]
  float normal_b = 0.002f;        ///< Quadratic coefficient [m⁻¹]
  float normal_c = 0.4f;          ///< Optimal depth [m]
  float lateral_factor = 0.001f;  ///< Lateral noise factor

  // Constant
  float constant_uncertainty = 0.1f;  ///< Fixed σ [m]
};

}  // namespace config
}  // namespace fastdem

#endif  // FASTDEM_CONFIG_SENSOR_MODEL_HPP
