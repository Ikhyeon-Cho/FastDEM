/*
 * factory.h
 *
 * Factory functions for creating core objects from configuration.
 * This file bridges config/ and core/ without coupling them directly.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_API_FACTORY_H
#define HEIGHT_MAPPING_API_FACTORY_H

#include <memory>

#include "height_mapping/config/estimation.h"
#include "height_mapping/config/map.h"
#include "height_mapping/config/sensor_model.h"
#include "height_mapping/core/map.h"
#include "height_mapping/core/updater/elevation_kalman.h"
#include "height_mapping/core/updater/elevation_quantile.h"
#include "height_mapping/core/updater/elevation_welford.h"
#include "height_mapping/sensors/uncertainty_model.h"

namespace height_mapping {

/**
 * @brief Create a HeightMap from configuration.
 *
 * @param config Map configuration
 * @return HeightMap instance (never null)
 */
inline std::unique_ptr<HeightMap> createHeightMap(const config::Map& config) {
  return std::make_unique<HeightMap>(config.width, config.height,
                                     config.resolution, config.frame_id);
}

/**
 * @brief Create a KalmanElevation updater from configuration.
 *
 * @param config Kalman filter configuration
 * @param alpha Fixed alpha for elevation blend
 * @return KalmanElevation updater
 */
inline updater::KalmanElevation createKalmanUpdater(
    const config::Kalman& config, float alpha) {
  return updater::KalmanElevation{config.min_variance, config.max_variance,
                                  config.process_noise, alpha};
}

/**
 * @brief Create a WelfordElevation updater from configuration.
 *
 * @param alpha Fixed alpha for elevation blend
 * @return WelfordElevation updater
 */
inline updater::WelfordElevation createWelfordUpdater(float alpha) {
  return updater::WelfordElevation{alpha};
}

/**
 * @brief Create a QuantileElevation updater from configuration.
 *
 * @param config P² quantile configuration
 * @return QuantileElevation updater
 */
inline updater::QuantileElevation createQuantileUpdater(
    const config::P2Quantile& config) {
  return updater::QuantileElevation{config.dn0,
                                    config.dn1,
                                    config.dn2,
                                    config.dn3,
                                    config.dn4,
                                    config.elevation_marker,
                                    config.max_sample_count};
}

/**
 * @brief Create an uncertainty model from configuration.
 *
 * @param config Sensor model configuration
 * @return Uncertainty model instance (never null)
 *
 * Supported types:
 * - "lidar": LiDARUncertaintyModel with range/angular noise based uncertainty
 * - "constant" (or any other): ConstantUncertaintyModel with fixed uncertainty
 *
 * @code
 *   auto model = createUncertaintyModel(config.sensor_model);
 *   // cloud must be centered at sensor, in gravity-aligned frame
 *   auto sigmas = model->computeUncertainties(cloud_map - sensor_origin_map);
 * @endcode
 */
inline std::unique_ptr<UncertaintyModel> createUncertaintyModel(
    const config::SensorModel& config) {
  if (config.type == "lidar") {
    return std::make_unique<LiDARUncertaintyModel>(
        config.range_noise, config.angular_noise, config.default_uncertainty);
  }
  // "constant" or any other type
  return std::make_unique<ConstantUncertaintyModel>(config.default_uncertainty);
}

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_API_FACTORY_H
