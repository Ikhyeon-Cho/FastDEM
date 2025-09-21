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
#include "height_mapping_core/data/point_cloud.h"
#include "pipeline_core/context.h"
#include <memory>

namespace height_mapping::core {
using PointCloudXYZ = height_map::PointCloudXYZ;
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
  MappingContext()
      : cloud_(std::make_shared<PointCloudXYZ>()),
        map_(std::make_shared<HeightMap>()) {}

  // Simple, consistent names for common access
  PointCloudXYZ &cloud() { return *cloud_; }
  const PointCloudXYZ &cloud() const { return *cloud_; }

  height_map::HeightMap &map() { return *map_; }
  const height_map::HeightMap &map() const { return *map_; }

  // Ownership management (less common operations)
  void replaceCloud(PointCloudXYZ::Ptr ptr) { cloud_ = ptr; }
  void replaceMap(std::shared_ptr<height_map::HeightMap> ptr) { map_ = ptr; }

  PointCloudXYZ::Ptr cloudPtr() const { return cloud_; }
  std::shared_ptr<height_map::HeightMap> mapPtr() const { return map_; }

  // Reset for new processing (keeps map)
  void reset() { cloud_->clear(); }

private:
  PointCloudXYZ::Ptr cloud_;
  std::shared_ptr<height_map::HeightMap> map_;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_MAPPING_CONTEXT_H