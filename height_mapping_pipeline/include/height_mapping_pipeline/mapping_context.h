/*
 * mapping_context.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PIPELINE_MAPPING_CONTEXT_H
#define HEIGHT_MAPPING_PIPELINE_MAPPING_CONTEXT_H

#include "height_mapping_core/data/height_map.h"
#include "height_mapping_core/data/point_cloud.h"
#include "flowpipe/context.h"
#include <memory>

namespace height_mapping::mapping {

/**
 * @brief Pipeline context for height mapping operations
 *
 * This class provides a workspace for the mapping pipeline,
 * carrying point cloud data and map reference through the processing stages.
 */
class MappingContext : public flowpipe::Context {
public:
  MappingContext(PointCloud::Ptr cloud, HeightMap::Ptr map)
      : cloud_(cloud), map_(map) {
    if (!cloud_) {
      throw std::invalid_argument("MappingContext: cloud cannot be null");
    }
    if (!map_) {
      throw std::invalid_argument("MappingContext: map cannot be null");
    }
  }

  // Data accessors
  PointCloud &cloud() { return *cloud_; }
  const PointCloud &cloud() const { return *cloud_; }
  PointCloud::Ptr cloudPtr() const { return cloud_; }

  HeightMap &map() { return *map_; }
  const HeightMap &map() const { return *map_; }
  HeightMap::Ptr mapPtr() const { return map_; }

  // Reset for new processing (keeps map)
  void reset() { cloud_->clear(); }

private:
  PointCloud::Ptr cloud_;
  HeightMap::Ptr map_;
};

} // namespace height_mapping::mapping

#endif // HEIGHT_MAPPING_PIPELINE_MAPPING_CONTEXT_H