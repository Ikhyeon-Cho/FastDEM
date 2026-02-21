// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * sensor_model.cpp
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fastdem/sensors/sensor_model.hpp"

#include <spdlog/spdlog.h>

#include "fastdem/sensors/lidar_model.hpp"
#include "fastdem/sensors/rgbd_model.hpp"

namespace fastdem {

std::unique_ptr<SensorModel> createSensorModel(const config::SensorModel& cfg) {
  switch (cfg.type) {
    case SensorType::LiDAR:
      return std::make_unique<LiDARSensorModel>(cfg.range_noise,
                                                cfg.angular_noise);
    case SensorType::RGBD:
      return std::make_unique<RGBDSensorModel>(
          cfg.normal_a, cfg.normal_b, cfg.normal_c, cfg.lateral_factor);
    case SensorType::Constant:
      return std::make_unique<ConstantUncertaintyModel>(
          cfg.constant_uncertainty);
    default:
      spdlog::warn("[SensorModel] Unknown type ({}), falling back to LiDAR",
                   static_cast<int>(cfg.type));
      return std::make_unique<LiDARSensorModel>(cfg.range_noise,
                                                cfg.angular_noise);
  }
}

}  // namespace fastdem
