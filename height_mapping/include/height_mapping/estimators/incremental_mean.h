/*
 * incremental_mean.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ESTIMATORS_INCREMENTAL_MEAN_H
#define HEIGHT_MAPPING_ESTIMATORS_INCREMENTAL_MEAN_H

#include "height_mapping/estimators/height_estimator_base.h"

namespace height_mapping::estimators {

/**
 * @brief Statistical mean estimator using Welford's online algorithm
 *
 * Computes running mean and variance using numerically stable method.
 * Reference:
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford's_online_algorithm
 */
class IncrementalMean : public HeightEstimatorBase {
 public:
  IncrementalMean() = default;

  void update(float &elevation, float &variance, float &count,
              float newMeasurement) override;

  std::string name() const override { return "IncrementalMean"; }
};

}  // namespace height_mapping::estimators

#endif  // HEIGHT_MAPPING_ESTIMATORS_INCREMENTAL_MEAN_H
