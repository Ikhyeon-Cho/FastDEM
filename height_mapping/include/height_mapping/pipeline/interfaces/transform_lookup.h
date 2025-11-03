/*
 * transform_lookup.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PIPELINE_INTERFACES_TRANSFORM_LOOKUP_H
#define HEIGHT_MAPPING_PIPELINE_INTERFACES_TRANSFORM_LOOKUP_H

#include <optional>
#include <string>

#include "height_mapping/core/data/transform.h"

namespace height_mapping {

/**
 * @brief Interface for looking up coordinate transforms
 *
 * This interface abstracts the source of transforms, allowing different
 * implementations (e.g., ROS TF2, static transforms, etc.)
 */
class TransformLookup {
 public:
  using Ptr = std::shared_ptr<TransformLookup>;
  using ConstPtr = std::shared_ptr<const TransformLookup>;

  virtual ~TransformLookup() = default;

  /**
   * @brief Look up transform between frames at a specific time
   * @param target_frame Target coordinate frame
   * @param source_frame Source coordinate frame
   * @param timestamp_ns Timestamp in nanoseconds
   * @return Transform if available, nullopt otherwise
   */
  virtual std::optional<Transform3D> lookupTransform(
      const std::string &target_frame, const std::string &source_frame,
      uint64_t timestamp_ns) = 0;

  /**
   * @brief Look up the latest available transform between frames
   * @param target_frame Target coordinate frame
   * @param source_frame Source coordinate frame
   * @return Transform if available, nullopt otherwise
   */
  virtual std::optional<Transform3D> lookupLatestTransform(
      const std::string &target_frame, const std::string &source_frame) = 0;
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_PIPELINE_INTERFACES_TRANSFORM_LOOKUP_H