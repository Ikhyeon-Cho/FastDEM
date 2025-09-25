/*
 * height_estimator_base.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ESTIMATORS_HEIGHT_ESTIMATOR_BASE_H
#define HEIGHT_MAPPING_CORE_ESTIMATORS_HEIGHT_ESTIMATOR_BASE_H

#include <string>

namespace height_mapping::estimators {

class HeightEstimatorBase {
public:
  virtual ~HeightEstimatorBase() = default;

  /**
   * @brief Update height estimate with a new measurement
   *
   * @param elevation Current elevation estimate (updated in-place)
   * @param variance Current variance estimate (updated in-place)
   * @param count Current measurement count (updated in-place)
   * @param newMeasurement New elevation measurement to incorporate
   */
  virtual void update(float &elevation, float &variance, float &count,
                      float newMeasurement) = 0;

  /**
   * @brief Get the name of this estimator
   */
  virtual std::string name() const = 0;

  /**
   * @brief Reset estimator state if any (optional)
   */
  virtual void reset() {}
};

} // namespace height_mapping::estimators

#endif // HEIGHT_MAPPING_CORE_ESTIMATORS_HEIGHT_ESTIMATOR_BASE_H