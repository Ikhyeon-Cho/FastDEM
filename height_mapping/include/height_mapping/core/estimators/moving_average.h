/*
 * moving_average.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ESTIMATORS_MOVING_AVERAGE_H
#define HEIGHT_MAPPING_CORE_ESTIMATORS_MOVING_AVERAGE_H

#include "height_mapping/core/estimators/height_estimator_base.h"

namespace height_mapping::estimators {

/**
 * @brief Exponential moving average estimator
 *
 * Uses exponentially weighted moving average for smooth height updates.
 */
class MovingAverage : public HeightEstimatorBase {
 public:
  struct Parameters {
    float alpha = 0.3f;  // Weight for new measurement [0,1]
                         // Higher alpha = more weight to new measurements
  };

  MovingAverage() : params_() {}
  explicit MovingAverage(const Parameters &params) : params_(params) {}

  void update(float &elevation, float &variance, float &count,
              float newMeasurement) override;

  std::string name() const override { return "MovingAverage"; }

  void setParameters(const Parameters &params) { params_ = params; }

 private:
  Parameters params_;
};

}  // namespace height_mapping::estimators

#endif  // HEIGHT_MAPPING_CORE_ESTIMATORS_MOVING_AVERAGE_H