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
      return std::make_unique<LiDARSensorModel>(cfg.lidar.range_noise,
                                                cfg.lidar.angular_noise);
    case SensorType::RGBD:
      return std::make_unique<RGBDSensorModel>(
          cfg.rgbd.normal_a, cfg.rgbd.normal_b, cfg.rgbd.normal_c,
          cfg.rgbd.lateral_factor);
    case SensorType::Constant:
      return std::make_unique<ConstantUncertaintyModel>(
          cfg.constant.uncertainty);
    default:
      spdlog::warn("[SensorModel] Unknown type ({}), falling back to LiDAR",
                   static_cast<int>(cfg.type));
      return std::make_unique<LiDARSensorModel>(cfg.lidar.range_noise,
                                                cfg.lidar.angular_noise);
  }
}

}  // namespace fastdem
