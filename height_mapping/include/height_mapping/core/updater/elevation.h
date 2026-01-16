/*
 * elevation.h
 *
 * Elevation layer updater wrapping estimator::Base.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_UPDATER_ELEVATION_H
#define HEIGHT_MAPPING_CORE_UPDATER_ELEVATION_H

#include <nanopcl/core.hpp>

#include "height_mapping/core/estimator/base.h"
#include "height_mapping/core/layers.h"
#include "height_mapping/core/map.h"

namespace height_mapping::updater {

/**
 * @brief Elevation layer updater using pluggable estimators.
 *
 * Wraps an estimator::Base to provide height estimation updates.
 * Also computes derived statistics (standard error, confidence interval)
 * in the finalize phase using Eigen vectorized operations.
 *
 * Required layers: elevation, variance, count
 * Derived layers: standard_error, confidence_interval_95
 *
 * Usage:
 * @code
 *   estimator::IncrementalMean estimator;
 *   updater::Elevation elev(estimator);
 *   height_mapping::update(map, cloud, elev);
 * @endcode
 */
class Elevation {
 public:
  /**
   * @brief Construct with an estimator.
   *
   * @param estimator Height estimator (IncrementalMean, KalmanFilter, etc.)
   *                  Must outlive this Elevation instance.
   */
  explicit Elevation(estimator::Base& estimator) : estimator_(estimator) {}

  /**
   * @brief Initialize elevation-related layers.
   *
   * Creates elevation, variance, count layers if missing.
   * Caches matrix references for fast access during updates.
   *
   * @param map Height map to initialize layers on
   */
  void initialize(HeightMap& map) {
    // Core layers
    if (!map.exists(layer::elevation)) map.add(layer::elevation, NAN);
    if (!map.exists(layer::variance)) map.add(layer::variance, 0.0f);
    if (!map.exists(layer::count)) map.add(layer::count, 0.0f);

    // Cache matrix references for fast access
    elevation_mat_ = &map.get(layer::elevation);
    variance_mat_ = &map.get(layer::variance);
    count_mat_ = &map.get(layer::count);
  }

  /**
   * @brief Update elevation estimate at a cell.
   *
   * Uses the pre-computed index to access cell values directly.
   * Delegates to the underlying estimator for the actual computation.
   *
   * @tparam PointT Point type (npcl::Point, npcl::ConstPointRef, etc.)
   * @param map Height map (unused, matrices are cached)
   * @param index Pre-computed cell index
   * @param point Point with z-coordinate as measurement
   */
  template <typename PointT>
  void update([[maybe_unused]] HeightMap& map, const grid_map::Index& index,
              const PointT& point) {
    float& elev = (*elevation_mat_)(index(0), index(1));
    float& var = (*variance_mat_)(index(0), index(1));
    float& cnt = (*count_mat_)(index(0), index(1));

    estimator_.update(elev, var, cnt, point.z());
  }

  /**
   * @brief Compute derived statistics after all updates.
   *
   * Batch-computes standard error and 95% confidence interval
   * using Eigen vectorized operations for efficiency.
   *
   * @param map Height map to add derived layers to
   */
  void finalize(HeightMap& map) {
    // Derived statistics layers
    if (!map.exists(layer::standard_error)) map.add(layer::standard_error, NAN);
    if (!map.exists(layer::confidence_interval))
      map.add(layer::confidence_interval, NAN);

    auto& se_mat = map.get(layer::standard_error);
    auto& ci_mat = map.get(layer::confidence_interval);

    // Vectorized computation: SE = sqrt(variance / count)
    se_mat = (variance_mat_->array() / count_mat_->array()).sqrt();

    // CI_95 = 1.96 * SE
    ci_mat = 1.96f * se_mat.array();
  }

  /**
   * @brief Get the underlying estimator.
   */
  estimator::Base& estimator() { return estimator_; }
  const estimator::Base& estimator() const { return estimator_; }

 private:
  estimator::Base& estimator_;

  // Cached matrix pointers (set during initialize)
  grid_map::Matrix* elevation_mat_ = nullptr;
  grid_map::Matrix* variance_mat_ = nullptr;
  grid_map::Matrix* count_mat_ = nullptr;
};

}  // namespace height_mapping::updater

#endif  // HEIGHT_MAPPING_CORE_UPDATER_ELEVATION_H
