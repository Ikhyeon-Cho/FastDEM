/*
 * incremental_mean.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/estimators/incremental_mean.h"
#include <cmath>

namespace height_mapping::core::estimators {

void IncrementalMean::update(float &elevation, float &variance, float &count,
                             float new_measurement) {
  // First measurement - initialize
  if (std::isnan(elevation)) {
    elevation = new_measurement;
    variance = 0.0f; // Undefined variance with single measurement
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
    // First variance calculation (n=2): variance = M2/(n-1) = M2/1
    variance = delta * delta2;
  } else {
    // Update existing variance (n>2)
    // Recover M2 from previous variance: M2_old = variance * (n-2)
    float m2 = variance * (count - 2.0f);
    m2 += delta * delta2;
    variance = m2 / (count - 1.0f);
  }

  // Update mean
  elevation = new_mean;
}

} // namespace height_mapping::core::estimators