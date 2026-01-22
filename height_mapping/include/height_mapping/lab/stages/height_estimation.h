/*
 * height_estimation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_LAB_STAGES_HEIGHT_ESTIMATION_H
#define HEIGHT_MAPPING_LAB_STAGES_HEIGHT_ESTIMATION_H

#include <spdlog/spdlog.h>

#include <memory>
#include <nanopcl/transform/transform_ops.hpp>
#include <string>
#include <variant>

#include "height_mapping/api/factory.h"
#include "height_mapping/config/estimation.h"
#include "height_mapping/core.h"
#include "height_mapping/core/updater/elevation_kalman.h"
#include "height_mapping/core/updater/elevation_quantile.h"
#include "height_mapping/core/updater/elevation_welford.h"
#include "height_mapping/core/updater/intensity.h"
#include "height_mapping/lab/frame.h"

namespace height_mapping::lab::stages {

/**
 * @brief Pipeline stage adapter for height estimation (header-only)
 */
class HeightEstimation : public ::ppl::Stage<MappingFrame> {
 public:
  using ElevationUpdater = std::variant<updater::WelfordElevation,
                                        updater::KalmanElevation,
                                        updater::QuantileElevation>;

  HeightEstimation() = default;

  void configure(const YAML::Node& config) override {
    std::string type = "kalman_filter";
    if (config["type"]) {
      type = config["type"].as<std::string>();
    }

    // Fixed alpha parameter
    float alpha = 0.5f;
    if (config["alpha"]) alpha = config["alpha"].as<float>();

    if (type == "welford") {
      elevation_updater_ = createWelfordUpdater(alpha);

    } else if (type == "p2_quantile") {
      config::P2Quantile p2_config;
      if (auto p = config["p2"]) {
        if (p["dn0"]) p2_config.dn0 = p["dn0"].as<float>();
        if (p["dn1"]) p2_config.dn1 = p["dn1"].as<float>();
        if (p["dn2"]) p2_config.dn2 = p["dn2"].as<float>();
        if (p["dn3"]) p2_config.dn3 = p["dn3"].as<float>();
        if (p["dn4"]) p2_config.dn4 = p["dn4"].as<float>();
        if (p["elevation_marker"])
          p2_config.elevation_marker = p["elevation_marker"].as<int>();
        if (p["max_sample_count"])
          p2_config.max_sample_count = p["max_sample_count"].as<float>();
      }
      elevation_updater_ = createQuantileUpdater(p2_config);

    } else {
      // Default: kalman_filter (1D Kalman filter with sensor model uncertainty)
      if (type != "kalman_filter") {
        spdlog::warn(
            "[HeightEstimation] Unknown estimator type: {}, using kalman_filter",
            type);
      }
      config::Kalman kalman_config;
      if (auto k = config["kalman"]) {
        if (k["min_variance"])
          kalman_config.min_variance = k["min_variance"].as<float>();
        if (k["max_variance"])
          kalman_config.max_variance = k["max_variance"].as<float>();
        if (k["process_noise"])
          kalman_config.process_noise = k["process_noise"].as<float>();
      }
      elevation_updater_ = createKalmanUpdater(kalman_config, alpha);
    }

    // Sensor model configuration - always create a sensor model
    float default_uncertainty = 0.1f;
    if (auto sm = config["sensor_model"]) {
      std::string sm_type = "constant";
      if (sm["type"]) sm_type = sm["type"].as<std::string>();
      // Treat deprecated aliases
      if (sm_type == "none") sm_type = "constant";
      if (sm_type == "laser") sm_type = "lidar";

      if (sm["default_uncertainty"])
        default_uncertainty = sm["default_uncertainty"].as<float>();

      if (sm_type == "lidar") {
        float range_noise = 0.02f;
        float angular_noise = 0.001f;
        if (sm["range_noise"]) range_noise = sm["range_noise"].as<float>();
        if (sm["angular_noise"]) angular_noise = sm["angular_noise"].as<float>();

        sensor_model_ = std::make_unique<LiDARUncertaintyModel>(
            range_noise, angular_noise, default_uncertainty);
        spdlog::debug("[HeightEstimation] sensor_model=lidar");
      } else {
        sensor_model_ = std::make_unique<ConstantUncertaintyModel>(default_uncertainty);
        spdlog::debug("[HeightEstimation] sensor_model=constant");
      }
    } else {
      // Default: constant uncertainty model
      sensor_model_ = std::make_unique<ConstantUncertaintyModel>(default_uncertainty);
      spdlog::debug("[HeightEstimation] sensor_model=constant (default)");
    }

    spdlog::debug("[HeightEstimation] estimator={}", type);
  }

  bool process(const std::shared_ptr<MappingFrame>& frame) override {
    auto& cloud = *frame->cloud;
    auto& map = frame->height_map;

    if (!map) {
      throw std::runtime_error("[HeightEstimation] HeightMap not available");
    }

    if (cloud.empty()) {
      return true;
    }

    // Transform to map frame if needed
    if (frame->robot_pose.isValid() &&
        cloud.frameId() != frame->robot_pose.parentFrame()) {
      cloud = npcl::transformCloud(std::move(cloud), frame->robot_pose);
    }

    // Compute sensor origin in map frame
    auto T_map_sensor = frame->robot_pose * frame->extrinsic;
    Eigen::Vector3f sensor_origin = T_map_sensor.translation();

    // Compute point uncertainties (σ) and convert to variances (σ²)
    auto uncertainties = sensor_model_->computeUncertainties(cloud - sensor_origin);
    auto variances = UncertaintyModel::toVariances(uncertainties);
    const bool has_variances = !variances.empty();

    // Run the core algorithm using variant updater with explicit loop
    std::visit([&](auto& elev_updater) {
      // 1. Initialize
      elev_updater.initialize(*map);
      intensity_updater_.initialize(*map);

      // 2. Loop: update each point
      for (const auto& point : cloud) {
        grid_map::Index index;
        if (!map->getIndex(grid_map::Position(point.x(), point.y()), index)) {
          continue;
        }
        const float point_var = has_variances ? variances[point.index()] : 0.0f;
        elev_updater.update(index, point.z(), point_var);
        if (point.hasIntensity()) {
          intensity_updater_.update(index, point.intensity());
        }
      }

      // 3. Finalize
      elev_updater.finalize();
    }, elevation_updater_);

    return true;
  }

 private:
  ElevationUpdater elevation_updater_{updater::KalmanElevation{}};
  updater::Intensity intensity_updater_;
  std::unique_ptr<UncertaintyModel> sensor_model_;
};

}  // namespace height_mapping::lab::stages
#endif  // HEIGHT_MAPPING_LAB_STAGES_HEIGHT_ESTIMATION_H
