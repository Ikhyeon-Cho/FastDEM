// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_TRANSFORM_PROVIDER_HPP
#define NANOPCL_TRANSFORM_PROVIDER_HPP

#include <memory>
#include <string>

#include "nanopcl/core/timestamp.hpp"
#include "nanopcl/transform/transform.hpp"

namespace nanopcl {

/**
 * @brief Interface for providing transforms between coordinate frames
 *
 * This interface abstracts the source of transforms, allowing different
 * implementations (e.g., ROS TF2, static transforms, test mocks, etc.)
 *
 * Dependency Injection pattern:
 * - Core library depends on this interface (abstract)
 * - Application provides concrete implementation (e.g., TF2Adapter)
 *
 * Example implementations:
 * - TF2Adapter: Queries ROS TF2 transform tree
 * - StaticProvider: Returns pre-configured static transforms
 * - MockProvider: For testing
 */
class TransformProvider {
 public:
  using Ptr = std::shared_ptr<TransformProvider>;
  using ConstPtr = std::shared_ptr<const TransformProvider>;

  virtual ~TransformProvider() = default;

  /**
   * @brief Get transform between frames at a specific time
   *
   * @param target_frame Target coordinate frame
   * @param source_frame Source coordinate frame
   * @param time Time to query transform (Timestamp: nanoseconds since epoch)
   * @return Frame-aware Transform object (target_frame <- source_frame)
   * @throws std::runtime_error if transform is not available
   */
  virtual Transform getTransform(const std::string& target_frame,
                                  const std::string& source_frame,
                                  Timestamp time) = 0;

  /**
   * @brief Get the latest available transform between frames
   *
   * @param target_frame Target coordinate frame
   * @param source_frame Source coordinate frame
   * @return Frame-aware Transform object (target_frame <- source_frame)
   * @throws std::runtime_error if transform is not available
   */
  virtual Transform getLatest(const std::string& target_frame,
                              const std::string& source_frame) = 0;

  /**
   * @brief Transform geometry object to target frame (high-level API)
   *
   * @tparam Geometry Geometry type (PointCloud, etc.)
   * @param geom_in Input geometry object (must have frameId() and timestamp())
   * @param target_frame Target coordinate frame
   * @return Transformed geometry in target frame
   * @throws std::runtime_error if transform is not available
   */
  template <typename Geometry>
  Geometry transform(const Geometry& geom_in, const std::string& target_frame) {
    const std::string& source_frame = geom_in.frameId();
    Timestamp time = geom_in.timestamp();
    Transform T_target_source = getTransform(target_frame, source_frame, time);
    return T_target_source * geom_in;
  }
};

}  // namespace nanopcl

#endif  // NANOPCL_TRANSFORM_PROVIDER_HPP
