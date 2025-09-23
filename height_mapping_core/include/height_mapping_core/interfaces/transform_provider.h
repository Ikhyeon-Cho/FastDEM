/*
 * transform_provider.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_INTERFACES_TRANSFORM_PROVIDER_H
#define HEIGHT_MAPPING_CORE_INTERFACES_TRANSFORM_PROVIDER_H

#include "height_mapping_core/geometry/transform.h"
#include <optional>
#include <string>

namespace height_mapping::core {

// Import Transform3D from geometry namespace
using geometry::Transform3D;

/**
 * @brief Interface for providing coordinate transforms
 *
 * This interface abstracts the source of transforms, allowing different
 * implementations (e.g., ROS TF2, static transforms, etc.)
 */
class ITransformProvider {
public:
  virtual ~ITransformProvider() = default;

  /**
   * @brief Look up transform between frames at a specific time
   * @param target_frame Target coordinate frame
   * @param source_frame Source coordinate frame
   * @param timestamp_ns Timestamp in nanoseconds
   * @return Transform if available, nullopt otherwise
   */
  virtual std::optional<Transform3D>
  lookupTransform(const std::string &target_frame,
                  const std::string &source_frame, uint64_t timestamp_ns) = 0;

  /**
   * @brief Look up the latest available transform between frames
   * @param target_frame Target coordinate frame
   * @param source_frame Source coordinate frame
   * @return Transform if available, nullopt otherwise
   */
  virtual std::optional<Transform3D>
  lookupLatestTransform(const std::string &target_frame,
                        const std::string &source_frame) = 0;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_INTERFACES_TRANSFORM_PROVIDER_H