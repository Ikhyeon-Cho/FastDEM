/*
 * moving_average.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/estimators/moving_average.h"

#include <algorithm>
#include <cmath>

namespace height_mapping::estimators {

void MovingAverage::update(float &elevation, float &variance, float &count,
                           float new_measurement) {
  // First measurement - initialize
  if (std::isnan(elevation)) {
    elevation = new_measurement;
    variance = 0.0f;
    count = 1.0f;
    return;
  }

  count += 1.0f;

  // Lambda for exponential weighted average
  const float alpha = std::max(0.01f, std::min(0.99f, params_.alpha));
  auto EWMA = [alpha](float old_value, float new_value) {
    return alpha * new_value + (1.0f - alpha) * old_value;
  };

  // Update elevation and variance using EWMA
  elevation = EWMA(elevation, new_measurement);
  const float deviation = new_measurement - elevation;
  variance = EWMA(variance, deviation * deviation);
}

}  // namespace height_mapping::estimators
