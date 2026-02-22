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

/// Sensor model parameters (grouped by sensor type)
struct SensorModel {
  SensorType type = SensorType::LiDAR;

  struct LiDAR {
    float range_noise = 0.02f;     ///< Range uncertainty σ_r [m]
    float angular_noise = 0.001f;  ///< Angular uncertainty σ_θ [rad]
  } lidar;

  struct RGBD {
    float normal_a = 0.001f;        ///< Base depth noise [m]
    float normal_b = 0.002f;        ///< Quadratic coefficient [m⁻¹]
    float normal_c = 0.4f;          ///< Optimal depth [m]
    float lateral_factor = 0.001f;  ///< Lateral noise factor
  } rgbd;

  struct Constant {
    float uncertainty = 0.03f;  ///< Fixed σ [m]
  } constant;
};

}  // namespace config
}  // namespace fastdem

#endif  // FASTDEM_CONFIG_SENSOR_MODEL_HPP
