/*
 * KalmanEstimator.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_mapping_core/estimators/HeightEstimatorBase.h"

namespace height_mapping {

class KalmanEstimator : public HeightEstimatorBase {
public:
  struct Parameters {
    float baseUncertainty{0.03f}; // Base measurement uncertainty
    float distanceFactor{0.001f}; // How much uncertainty increases with distance
    float angleFactor{0.001f};    // How much uncertainty increases with angle
    float minVariance{0.001f};    // Minimum allowed variance
    float maxVariance{1.0f};      // Maximum allowed variance
    bool adaptiveVariance{true};  // Enable distance/angle-based variance
  };

  explicit KalmanEstimator() = default;
  ~KalmanEstimator() override = default;

  void setParameters(const Parameters &params) { params_ = params; }

  void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZ> &cloud) override;
  void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZI> &cloud) override;
  void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZRGB> &cloud) override;

  /**
   * Kalman filter update step
   * @param height Current height estimate
   * @param variance Current height variance
   * @param pointHeight New height measurement
   * @param pointVariance Measurement uncertainty
   */
  static void
  kalmanUpdate(float &height, float &variance, float pointHeight, float pointVariance) {

    const float kalmanGain = variance / (variance + pointVariance);
    height = height + kalmanGain * (pointHeight - height);
    variance = (1.0f - kalmanGain) * variance;
  }

private:
  /**
   * Calculate measurement variance based on point properties
   * Considers distance and angle from sensor
   */
  float getPointVariance(const Eigen::Vector3f &point) const {
    if (!params_.adaptiveVariance) {
      return params_.baseUncertainty;
    }

    // Distance-based uncertainty
    const float distance = point.norm();
    float variance =
        params_.baseUncertainty + params_.distanceFactor * distance * distance;

    // Angle-based uncertainty (if point not directly below sensor)
    if (distance > 1e-3f) {
      const float angle = std::acos(std::abs(point.z()) / distance);
      variance += params_.angleFactor * angle * angle;
    }

    return clamp(variance, params_.minVariance, params_.maxVariance);
  }

  /**
   * Get normalized confidence score [0,1]
   * Higher values indicate more reliable estimates
   */
  float getConfidence(float variance) const {
    return 1.0f - clamp(variance / params_.maxVariance, 0.0f, 1.0f);
  }

  static float clamp(float value, float min, float max) {
    return std::min(std::max(value, min), max);
  }

  Parameters params_;
};

} // namespace height_mapping
