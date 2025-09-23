/*
 * estimator_factory.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ESTIMATORS_ESTIMATOR_FACTORY_H
#define HEIGHT_MAPPING_CORE_ESTIMATORS_ESTIMATOR_FACTORY_H

#include "height_mapping_core/estimators/height_estimator_base.h"
#include <map>
#include <memory>
#include <string>

namespace height_mapping::core::estimators {

/**
 * @brief Factory class for creating height estimators
 *
 * Centralizes estimator creation logic and parameter extraction
 */
class EstimatorFactory {
public:
  /**
   * @brief Create an estimator based on type and parameters
   *
   * @param type Estimator type: "kalman", "mean", "stat_mean", or
   * "moving_average"
   * @param params Configuration parameters for the estimator
   * @return Unique pointer to the created estimator
   */
  static std::unique_ptr<HeightEstimatorBase>
  create(const std::string &type,
         const std::map<std::string, std::string> &params);

private:
  // Helper to extract float parameter with default
  static float getFloatParam(const std::map<std::string, std::string> &params,
                             const std::string &key, float defaultValue);
};

} // namespace height_mapping::core::estimators

#endif // HEIGHT_MAPPING_CORE_ESTIMATORS_ESTIMATOR_FACTORY_H