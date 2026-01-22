/*
 * sensor_model.h
 *
 * Sensor model configuration for measurement uncertainty.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CONFIG_SENSOR_MODEL_H
#define HEIGHT_MAPPING_CONFIG_SENSOR_MODEL_H

#include <string>

namespace height_mapping::config {

/**
 * @brief Sensor model configuration for measurement uncertainty.
 *
 * For 3D LiDAR, the height (Z) uncertainty is computed as:
 *   σ_z² = (σ_r × z/d)² + (σ_θ × √(x² + y²))²
 *
 * Where:
 *   - σ_r = range noise [m]
 *   - σ_θ = angular noise [rad]
 *   - d = distance to point
 */
struct SensorModel {
  /// Sensor model type: "constant" or "lidar"
  std::string type = "constant";

  /// @name 3D LiDAR model parameters
  /// @{
  float range_noise = 0.02f;    ///< Range uncertainty σ_r [m]
  float angular_noise = 0.001f; ///< Angular uncertainty σ_θ [rad]
  /// @}

  /// Default uncertainty σ [m] (used by ConstantUncertaintyModel)
  float default_uncertainty = 0.1f;
};

}  // namespace height_mapping::config

#endif  // HEIGHT_MAPPING_CONFIG_SENSOR_MODEL_H
