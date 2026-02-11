// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * fastdem.hpp
 *
 * Configuration for FastDEM.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_CONFIG_FASTDEM_HPP
#define FASTDEM_CONFIG_FASTDEM_HPP

#include <string>

#include "fastdem/config/feature_extraction.hpp"
#include "fastdem/config/inpainting.hpp"
#include "fastdem/config/mapping.hpp"
#include "fastdem/config/rasterization.hpp"
#include "fastdem/config/raycasting.hpp"
#include "fastdem/config/scan_filter.hpp"
#include "fastdem/config/sensor_model.hpp"
#include "fastdem/config/uncertainty_fusion.hpp"

namespace fastdem {

/**
 * @brief Configuration for FastDEM.
 */
struct MappingConfig {
  config::Mapping mapping;
  config::ScanFilter scan_filter;
  config::Rasterization rasterization;
  config::Raycasting raycasting;
  config::Inpainting inpainting;
  config::Sensor sensor;
  config::UncertaintyFusion uncertainty_fusion;
  config::FeatureExtraction feature_extraction;

  static MappingConfig load(const std::string& path);
};

}  // namespace fastdem

#endif  // FASTDEM_CONFIG_FASTDEM_HPP
