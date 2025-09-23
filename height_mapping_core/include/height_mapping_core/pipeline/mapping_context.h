/*
 * mapping_context.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_MAPPING_CONTEXT_H
#define HEIGHT_MAPPING_CORE_PIPELINE_MAPPING_CONTEXT_H

#include "height_mapping_core/data/height_map.h"
#include "height_mapping_core/geometry/point_cloud.h"
#include "pipeline_core/context.h"
#include <memory>

namespace height_mapping::core {
using PointCloud = geometry::PointCloud;
using HeightMap = height_map::HeightMap;

/**
 * @brief Context for height mapping operations
 *
 * This class provides a workspace for the mapping pipeline,
 * carrying point cloud data and map reference through the processing stages.
 * The point cloud is transformed in-place by stages as needed.
 */
class MappingContext : public pipeline::Context {
public:
  // Constructor requires both cloud and map - no default constructor
  MappingContext(std::shared_ptr<PointCloud> cloud,
                 std::shared_ptr<HeightMap> map)
      : cloud_(cloud), map_(map) {
    if (!cloud_) {
      throw std::invalid_argument("MappingContext: cloud cannot be null");
    }
    if (!map_) {
      throw std::invalid_argument("MappingContext: map cannot be null");
    }
  }

  // Simple, consistent names for common access
  PointCloud &cloud() { return *cloud_; }
  const PointCloud &cloud() const { return *cloud_; }

  HeightMap &map() { return *map_; }
  const HeightMap &map() const { return *map_; }

  // Direct pointer access when needed
  std::shared_ptr<PointCloud> cloudPtr() const { return cloud_; }
  std::shared_ptr<HeightMap> mapPtr() const { return map_; }

  // Reset for new processing (keeps map)
  void reset() { cloud_->clear(); }

private:
  PointCloud::Ptr cloud_;
  std::shared_ptr<height_map::HeightMap> map_;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_MAPPING_CONTEXT_H