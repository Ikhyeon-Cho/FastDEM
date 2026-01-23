/*
 * uncertainty_model.h
 *
 * Sensor noise models for measurement uncertainty computation.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_SENSORS_UNCERTAINTY_MODEL_H
#define HEIGHT_MAPPING_SENSORS_UNCERTAINTY_MODEL_H

#include <Eigen/Core>
#include <cmath>
#include <nanopcl/core.hpp>
#include <vector>

namespace height_mapping {

// ============================================================================
// Declarations
// ============================================================================

/**
 * @brief Abstract base class for measurement uncertainty models.
 *
 * Computes measurement uncertainty (standard deviation) based on sensor
 * characteristics. Always returns σ (not σ²).
 *
 * Input points should be:
 *   - Centered at sensor origin (i.e., beam vectors from sensor to point)
 *   - In a gravity-aligned frame (Z-axis = up, XY-plane = horizontal)
 *
 * This ensures correct height uncertainty computation regardless of sensor
 * mounting angle. Use (cloud_map - sensor_origin_map) to get proper input.
 */
class UncertaintyModel {
 public:
  virtual ~UncertaintyModel() = default;

  /**
   * @brief Compute height (Z) uncertainty for a single point.
   *
   * @param point Beam vector from sensor to point, in gravity-aligned frame
   * @return Height uncertainty σ_z (standard deviation, not variance)
   */
  virtual float computeUncertainty(const Eigen::Vector3f& point) const = 0;

  /**
   * @brief Compute height uncertainties for all points in a cloud.
   *
   * @param cloud Point cloud centered at sensor origin, in gravity-aligned frame
   * @return Per-point height uncertainties σ_z (standard deviation)
   *
   * @code
   *   LiDARUncertaintyModel model(0.02f, 0.001f);
   *   auto cloud_centered = cloud_map - sensor_origin_map;
   *   auto sigmas = model.computeUncertainties(cloud_centered);
   * @endcode
   */
  std::vector<float> computeUncertainties(const npcl::PointCloud& cloud) const;

  /**
   * @brief Convert uncertainties (σ) to variances (σ²).
   */
  static std::vector<float> toVariances(const std::vector<float>& uncertainties);
};

/**
 * @brief Constant uncertainty model.
 *
 * Returns a fixed uncertainty value regardless of point position.
 * Used as default when no specific sensor model is configured.
 */
class ConstantUncertaintyModel : public UncertaintyModel {
 public:
  explicit ConstantUncertaintyModel(float uncertainty = 0.1f);
  float computeUncertainty(const Eigen::Vector3f& point) const override;

 private:
  float uncertainty_;  ///< Fixed uncertainty σ [m]
};

/**
 * @brief 3D LiDAR uncertainty model for height mapping.
 *
 * Computes the vertical (Z) uncertainty of a LiDAR measurement based on:
 *   - Range noise (σ_r): uncertainty along the beam direction
 *   - Angular noise (σ_θ): uncertainty perpendicular to the beam
 *
 * For a point at (x, y, z) in sensor frame with distance d = √(x² + y² + z²):
 *   - Elevation angle: θ = asin(z/d)
 *   - Z = d × sin(θ)
 *
 * Height uncertainty propagation:
 *   σ_z² = (∂z/∂d × σ_r)² + (∂z/∂θ × σ_θ)²
 *        = (sin(θ) × σ_r)² + (d × cos(θ) × σ_θ)²
 *        = (z/d × σ_r)² + (√(x² + y²) × σ_θ)²
 *
 * Typical values:
 *   - σ_r: 0.01-0.03m (range accuracy from datasheet)
 *   - σ_θ: 0.001-0.003 rad (~0.05-0.17°, angular resolution)
 */
class LiDARUncertaintyModel : public UncertaintyModel {
 public:
  /**
   * @brief Construct with LiDAR noise parameters.
   *
   * @param range_noise Range uncertainty σ_r [m]
   * @param angular_noise Angular uncertainty σ_θ [rad]
   * @param default_uncertainty Fallback for degenerate cases [m]
   */
  LiDARUncertaintyModel(float range_noise = 0.02f, float angular_noise = 0.001f,
                        float default_uncertainty = 0.1f);

  float computeUncertainty(const Eigen::Vector3f& point) const override;

 private:
  float range_noise_;          ///< Range uncertainty σ_r [m]
  float angular_noise_;        ///< Angular uncertainty σ_θ [rad]
  float default_uncertainty_;  ///< Fallback for degenerate cases [m]
};

// ============================================================================
// Inline Implementations
// ============================================================================

inline std::vector<float> UncertaintyModel::computeUncertainties(
    const npcl::PointCloud& cloud) const {
  std::vector<float> result;
  result.reserve(cloud.size());
  for (const auto& pt : cloud) {
    result.push_back(computeUncertainty(pt.xyz()));
  }
  return result;
}

inline std::vector<float> UncertaintyModel::toVariances(
    const std::vector<float>& uncertainties) {
  std::vector<float> variances;
  variances.reserve(uncertainties.size());
  for (float sigma : uncertainties) {
    variances.push_back(sigma * sigma);
  }
  return variances;
}

inline ConstantUncertaintyModel::ConstantUncertaintyModel(float uncertainty)
    : uncertainty_(uncertainty) {}

inline float ConstantUncertaintyModel::computeUncertainty(
    const Eigen::Vector3f& /*point*/) const {
  return uncertainty_;
}

inline LiDARUncertaintyModel::LiDARUncertaintyModel(float range_noise,
                                                    float angular_noise,
                                                    float default_uncertainty)
    : range_noise_(range_noise),
      angular_noise_(angular_noise),
      default_uncertainty_(default_uncertainty) {}

inline float LiDARUncertaintyModel::computeUncertainty(
    const Eigen::Vector3f& point) const {
  const float x = point.x();
  const float y = point.y();
  const float z = point.z();

  const float distance_sq = point.squaredNorm();
  if (distance_sq < 1e-6f) {
    return default_uncertainty_;
  }

  const float distance = std::sqrt(distance_sq);
  const float horizontal_dist = std::sqrt(x * x + y * y);

  // Height uncertainty from range noise: (z/d) × σ_r
  const float z_from_range = (z / distance) * range_noise_;

  // Height uncertainty from angular noise: √(x² + y²) × σ_θ
  const float z_from_angle = horizontal_dist * angular_noise_;

  // Combined uncertainty (independent sources)
  const float variance =
      z_from_range * z_from_range + z_from_angle * z_from_angle;
  return std::sqrt(variance);
}

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_SENSORS_UNCERTAINTY_MODEL_H
