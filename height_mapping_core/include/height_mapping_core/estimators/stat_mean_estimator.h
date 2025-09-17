/*
 * StatMeanEstimator.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_mapping_core/estimators/height_estimator_base.h"

namespace height_mapping {
class StatMeanEstimator : public HeightEstimatorBase {
public:
  StatMeanEstimator() = default;
  ~StatMeanEstimator() override = default;

  void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZ> &cloud) override;
  void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZI> &cloud) override;
  void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZRGB> &cloud) override;

  std::string getType() const override { return "StatMean"; }

private:
  /**
   * Updates running statistics (mean, variance) using Welford's online
   * algorithm Reference:
   * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
   */
  void updateHeightStats(float &height, float &variance, float n, float new_height) {

    const float delta = (new_height - height);
    const float delta_n = delta / n;

    // Update mean
    height += delta_n;

    // Update variance using the more stable algorithm
    // M2 = M2 + delta * (new_value - updated_mean)
    // where M2 is the sum of squared differences from the mean
    variance += delta * (new_height - height);

    // Convert M2 to variance
    if (n > 1) {
      variance = variance / (n - 1); // Use (n-1) for sample variance
    }
  }

  /**
   * Updates running mean using numerically stable method
   * Handles potential numerical precision issues with large n
   */
  void meanFilter(float &mean, float n, float new_value) {
    mean += (new_value - mean) / n;
  }

  /**
   * Calculate Standard Error of the Mean (SEM)
   * SEM = σ/√n, where σ is standard deviation and n is sample size
   * Lower SEM indicates more precise estimate of the true mean
   */
  float getStandardError(float n, float variance) const {
    if (n < 2)
      return std::numeric_limits<float>::infinity();
    return std::sqrt(variance / n);
  }

  /**
   * Calculate confidence interval using t-distribution
   * For 95% confidence level with degrees of freedom = n-1
   * Returns the half-width of the confidence interval
   */
  float
  getConfidenceInterval(float n, float variance, float confidence_level = 0.95f) const {
    if (n < 2)
      return std::numeric_limits<float>::infinity();

    // t-value for 95% confidence level (could be made configurable)
    // This is simplified; ideally would use actual t-distribution quantiles
    const float t_value = 1.96f; // Approximation using normal distribution

    return t_value * getStandardError(n, variance);
  }

  /**
   * Calculate relative standard error (RSE) as a percentage
   * RSE = (SEM/mean) * 100
   * Useful for comparing variability between measurements
   */
  float getRelativeStandardError(float mean, float n, float variance) const {
    if (std::abs(mean) < 1e-6f || n < 2)
      return std::numeric_limits<float>::infinity();

    return (getStandardError(n, variance) / std::abs(mean)) * 100.0f;
  }
};
} // namespace height_mapping
