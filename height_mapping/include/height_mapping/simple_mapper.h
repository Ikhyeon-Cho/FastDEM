/*
 * simple_mapper.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <memory>
#include <shared_mutex>
#include <string>

#include "height_mapping/core/algorithms/height_estimation.h"
#include "height_mapping/core/algorithms/transform_cloud.h"
#include "height_mapping/core/data/height_map.h"
#include "height_mapping/core/data/point_cloud.h"
#include "height_mapping/pipeline/interfaces/config.h"
#include "height_mapping/pipeline/interfaces/transform_lookup.h"

namespace height_mapping {

/**
 * @brief Minimal baseline implementation for height mapping
 *
 * This is a lightweight, pipeline-free implementation that performs only
 * essential operations for height mapping research:
 * 1. Transform point cloud to map frame
 * 2. Height estimation using incremental mean
 *
 * Features:
 * - No pipeline overhead (direct algorithm calls)
 * - Minimal dependencies (no flowpipelib required)
 * - Thread-safe access to height map
 * - Suitable as performance baseline for research
 *
 * Limitations:
 * - No filtering stages (voxel, passthrough, etc.)
 * - No ground segmentation
 * - No raycasting
 * - Fixed estimator type (incremental mean)
 *
 * Use HeightMapper for configurable pipeline-based processing.
 */
class SimpleHeightMapper {
 public:
  using Ptr = std::unique_ptr<SimpleHeightMapper>;

  // Constructor with optional config
  SimpleHeightMapper(TransformLookup::Ptr tf, Config config = Config());

  ~SimpleHeightMapper() = default;

  // User interfaces
  void integrate(std::shared_ptr<PointCloud> cloud);
  HeightMap::ConstPtr getHeightMap() const;
  std::shared_ptr<const PointCloud> getScanProcessed() const;
  void reset();

 private:
  HeightMap::Ptr initMap(const Config &config);

  Config cfg_;
  TransformLookup::Ptr tf_tree_;
  HeightMap::Ptr map_;

  // Core algorithms (no pipeline)
  algorithms::TransformCloud transformer_;
  algorithms::HeightEstimation estimator_;

  mutable std::shared_mutex map_mutex_;
  mutable std::shared_mutex cloud_mutex_;
  std::shared_ptr<PointCloud> last_processed_cloud_;
};

}  // namespace height_mapping
