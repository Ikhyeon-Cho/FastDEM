#pragma once

#include <string>

#include "height_mapping/types.h"

namespace height_mapping::segmenters {

class GroundSegmenterBase {
 public:
  virtual ~GroundSegmenterBase() = default;

  /**
   * @brief Segment point cloud into ground and obstacle points.
   *
   * @param input Input point cloud (const, not modified)
   * @param ground Output: points classified as ground
   * @param obstacles Output: points classified as obstacles
   */
  virtual void segment(const PointCloud& input, PointCloud& ground,
                       PointCloud& obstacles) = 0;

  /**
   * @brief Get the name of this segmenter.
   */
  virtual std::string name() const = 0;
};

}  // namespace height_mapping::segmenters
