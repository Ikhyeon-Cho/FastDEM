/*
 * incremental_mean.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ESTIMATOR_INCREMENTAL_MEAN_H
#define HEIGHT_MAPPING_CORE_ESTIMATOR_INCREMENTAL_MEAN_H

#include <cmath>

#include "height_mapping/core/estimator/base.h"

namespace height_mapping::estimator {

/**
 * @brief Statistical mean estimator using Welford's online algorithm
 *
 * Computes running mean and variance using numerically stable method.
 * Reference:
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
 */
class IncrementalMean : public Base {
 public:
  IncrementalMean() = default;

  void update(float& elevation, float& variance, float& count,
              float new_measurement) override {
    // First measurement - initialize
    if (std::isnan(elevation)) {
      elevation = new_measurement;
      variance = 0.0f;
      count = 1.0f;
      return;
    }

    count += 1.0f;

    // Welford's online algorithm for mean and variance
    const float delta = new_measurement - elevation;
    const float new_mean = elevation + (delta / count);
    const float delta2 = new_measurement - new_mean;

    // Update variance using stable computation
    if (count <= 2.0f) {
      variance = delta * delta2;
    } else {
      float m2 = variance * (count - 2.0f);
      m2 += delta * delta2;
      variance = m2 / (count - 1.0f);
    }

    elevation = new_mean;
  }

  std::string name() const override { return "IncrementalMean"; }
};

}  // namespace height_mapping::estimator

#endif  // HEIGHT_MAPPING_CORE_ESTIMATOR_INCREMENTAL_MEAN_H
