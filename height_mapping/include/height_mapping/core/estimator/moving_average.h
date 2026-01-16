/*
 * moving_average.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ESTIMATOR_MOVING_AVERAGE_H
#define HEIGHT_MAPPING_CORE_ESTIMATOR_MOVING_AVERAGE_H

#include <algorithm>
#include <cmath>

#include "height_mapping/core/estimator/base.h"

namespace height_mapping::estimator {

/**
 * @brief Exponential moving average estimator
 *
 * Uses exponentially weighted moving average for smooth height updates.
 */
class MovingAverage : public Base {
 public:
  MovingAverage() = default;
  explicit MovingAverage(float alpha) : alpha_(alpha) {}

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

    // Clamp alpha to valid range
    const float alpha = std::clamp(alpha_, 0.01f, 0.99f);

    // Exponential weighted moving average
    elevation = alpha * new_measurement + (1.0f - alpha) * elevation;
    const float deviation = new_measurement - elevation;
    variance = alpha * (deviation * deviation) + (1.0f - alpha) * variance;
  }

  std::string name() const override { return "MovingAverage"; }

 private:
  float alpha_ = 0.3f;  // Weight for new measurement [0,1]
};

}  // namespace height_mapping::estimator

#endif  // HEIGHT_MAPPING_CORE_ESTIMATOR_MOVING_AVERAGE_H
