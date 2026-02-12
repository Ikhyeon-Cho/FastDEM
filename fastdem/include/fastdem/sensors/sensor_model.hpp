// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * sensor_model.hpp
 *
 * Base class and common sensor models for measurement uncertainty computation.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_SENSORS_SENSOR_MODEL_HPP
#define FASTDEM_SENSORS_SENSOR_MODEL_HPP

#include <Eigen/Core>
#include <cmath>
#include <memory>

#include "fastdem/config/sensor_model.hpp"
#include "fastdem/point_types.hpp"

namespace fastdem {

/**
 * @brief Abstract base class for sensor models.
 *
 * Sensor models compute measurement uncertainty based on sensor
 * characteristics. The interface is covariance-based:
 *
 *   1. computeSensorCovariance(point_sensor) → 3x3 covariance in sensor frame
 *   2. computeSensorCovariances(cloud) → batch version, stores in cloud
 *
 * Use projectToHeightVariance() (free function) to get σ_z² in map frame.
 */
class SensorModel {
 public:
  virtual ~SensorModel() = default;

  /**
   * @brief Compute 3x3 covariance matrix in sensor frame.
   *
   * @param point_sensor Point position in sensor frame
   * @return Covariance matrix Σ_sensor (3x3, symmetric positive semi-definite)
   */
  virtual Eigen::Matrix3f computeSensorCovariance(
      const Eigen::Vector3f& point_sensor) const = 0;

  /**
   * @brief Compute and store covariances for entire cloud.
   *
   * Enables covariance channel in the cloud and populates it.
   *
   * @param cloud_sensor Point cloud in sensor frame (modified in place)
   */
  virtual void computeSensorCovariances(PointCloud& cloud_sensor) const;
};

/**
 * @brief Constant (isotropic) uncertainty model.
 *
 * Returns a fixed uncertainty regardless of point position.
 * Used as default when no specific sensor model is configured.
 */
class ConstantUncertaintyModel : public SensorModel {
 public:
  explicit ConstantUncertaintyModel(float uncertainty = 0.1f);

  Eigen::Matrix3f computeSensorCovariance(
      const Eigen::Vector3f& p) const override;

 private:
  float variance_;
};

inline void SensorModel::computeSensorCovariances(PointCloud& scan) const {
  scan.useCovariance();
  auto& covs = scan.covariances();

  for (size_t i = 0; i < scan.size(); ++i) {
    covs[i] = computeSensorCovariance(scan.point(i));
  }
}

inline ConstantUncertaintyModel::ConstantUncertaintyModel(float uncertainty)
    : variance_(uncertainty * uncertainty) {}

inline Eigen::Matrix3f ConstantUncertaintyModel::computeSensorCovariance(
    const Eigen::Vector3f& /*point_sensor*/) const {
  return Eigen::Matrix3f::Identity() * variance_;
}

/// Factory: create sensor model from config
std::unique_ptr<SensorModel> createSensorModel(const config::SensorModel& cfg);

}  // namespace fastdem

#endif  // FASTDEM_SENSORS_SENSOR_MODEL_HPP
