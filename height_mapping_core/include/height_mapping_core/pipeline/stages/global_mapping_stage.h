/*
 * global_mapping_stage.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_STAGES_GLOBAL_MAPPING_STAGE_H
#define HEIGHT_MAPPING_CORE_PIPELINE_STAGES_GLOBAL_MAPPING_STAGE_H

#include "height_mapping_core/pipeline/processing_context.h"
#include "height_mapping_core/pipeline/stage.h"
#include <grid_map_core/GridMap.hpp>
#include <unordered_set>

// Enable unordered_set for grid_map::Index
namespace std {
template <> struct hash<grid_map::Index> {
  std::size_t operator()(const grid_map::Index &index) const {
    std::size_t h1 = std::hash<int>{}(index[0]);
    std::size_t h2 = std::hash<int>{}(index[1]);
    return h1 ^ (h2 << 1);
  }
};

template <> struct equal_to<grid_map::Index> {
  bool operator()(const grid_map::Index &lhs,
                  const grid_map::Index &rhs) const {
    return (lhs[0] == rhs[0]) && (lhs[1] == rhs[1]);
  }
};
} // namespace std

namespace height_mapping::core {

class GlobalMappingStage : public PipelineStage<GlobalMappingStage> {
public:
  static constexpr const char *STAGE_NAME = "GlobalMapping";
  static constexpr const char *STAGE_TYPE = "Map Tracking";

  GlobalMappingStage() = default;

  void initialize() override {
    initialized_ = true;
    measured_indices_.clear();
  }

  void processImpl(ProcessingContext &ctx) {
    if (ctx.cloud().empty()) {
      return;
    }

    auto &map = ctx.map();
    if (!map.isInitialized()) {
      return;
    }

    // Track which cells have been measured
    trackMeasuredCells(map, ctx.cloud());

    // Add tracking info to ctx
    ctx.set("measured_cells_count",
              static_cast<int>(measured_indices_.size()));
    ctx.set("total_cells", map.getSize()(0) * map.getSize()(1));

    // Calculate coverage percentage
    float coverage = 100.0f * measured_indices_.size() /
                     (map.getSize()(0) * map.getSize()(1));
    ctx.set("map_coverage_percentage", coverage);
  }

  bool canProcess(const ProcessingContext &ctx) const override {
    return !frame.cloud().empty() && initialized_;
  }

  // Get all measured grid indices
  const std::unordered_set<grid_map::Index> &getMeasuredIndices() const {
    return measured_indices_;
  }

  // Clear measured indices (for reset)
  void clearMeasuredIndices() { measured_indices_.clear(); }

  // Check if a specific cell has been measured
  bool isMeasured(const grid_map::Index &index) const {
    return measured_indices_.find(index) != measured_indices_.end();
  }

  // Get measured cells as point cloud for visualization/saving
  pcl::PointCloud<pcl::PointXYZ>
  getMeasuredCellsAsCloud(const height_map::HeightMap &map) const {
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (const auto &index : measured_indices_) {
      grid_map::Position position;
      if (!map.getPosition(index, position))
        continue;

      float height = map.getHeight(position.x(), position.y());
      if (!std::isfinite(height))
        continue;

      pcl::PointXYZ point;
      point.x = position.x();
      point.y = position.y();
      point.z = height;
      cloud.points.push_back(point);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = false;

    return cloud;
  }

private:
  void trackMeasuredCells(const height_map::HeightMap &map,
                          const pcl::PointCloud<pcl::PointXYZ> &cloud) {
    grid_map::Index index;
    grid_map::Position position;

    for (const auto &point : cloud.points) {
      if (!point.isFinite())
        continue;

      position << point.x, point.y;
      if (!map.getIndex(position, index))
        continue;

      // Add to measured set
      measured_indices_.insert(index);
    }
  }

  std::unordered_set<grid_map::Index> measured_indices_;
  bool initialized_ = false;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_STAGES_GLOBAL_MAPPING_STAGE_H