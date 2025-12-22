/*
 * extrinsics_provider.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_INTERFACES_EXTRINSICS_PROVIDER_H
#define HEIGHT_MAPPING_CORE_INTERFACES_EXTRINSICS_PROVIDER_H

#include <memory>
#include <optional>
#include <string>

#include "height_mapping/types.h"

namespace height_mapping {

/**
 * @brief Interface for providing sensor extrinsic calibration
 *
 * This interface provides the static transformation from sensor frame
 * to robot body frame (base_link). This is typically the result of
 * sensor calibration and remains constant during operation.
 *
 * Semantic meaning: "Where is the sensor mounted on the robot?"
 */
class IExtrinsicsProvider {
 public:
  using Ptr = std::shared_ptr<IExtrinsicsProvider>;
  using ConstPtr = std::shared_ptr<const IExtrinsicsProvider>;

  virtual ~IExtrinsicsProvider() = default;

  /**
   * @brief Get the extrinsic transform from sensor to robot body frame
   * @param sensor_frame The sensor coordinate frame name
   * @return Transform T_base_sensor (Sensor → Base) if available
   *
   * The returned transform converts points from sensor frame to base_link.
   * This is typically a static calibration result.
   */
  virtual std::optional<Transformf> getExtrinsic(
      const std::string& sensor_frame) = 0;

  /**
   * @brief Get the robot body frame name (e.g., "base_link", "base_footprint")
   */
  virtual std::string getBaseFrame() const = 0;
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_CORE_INTERFACES_EXTRINSICS_PROVIDER_H
