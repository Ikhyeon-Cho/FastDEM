/*
 * voxel_filter.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/voxel_filter.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registry.h"

#include <unordered_map>
#include <cmath>
#include <iostream>

namespace height_mapping::core {

VoxelFilterStage::VoxelFilterStage()
    : Stage("VoxelFilter", "Point Cloud Filter") {
}

void VoxelFilterStage::configure(const std::map<std::string, std::string>& params) {
  auto it = params.find("voxel_size");
  if (it != params.end()) {
    float size = std::stof(it->second);

    // Validate voxel size
    if (size <= 0.0f || size > 10.0f) {
      std::cerr << "[VoxelFilter] Warning: Invalid voxel size " << size
                << ", using default " << voxel_size_ << std::endl;
      return;
    }

    voxel_size_ = size;
  }
}

void VoxelFilterStage::processImpl(pipeline::Context& ctx) {
  // Safe cast - we know this is a MappingContext in our pipeline
  auto& mapping_ctx = static_cast<MappingContext&>(ctx);

  auto& cloud = mapping_ctx.cloud();
  if (cloud.empty()) {
    return;
  }

  size_t points_before = cloud.size();

  // Simple voxel grid filtering implementation
  // Using spatial hashing to group points into voxels
  struct VoxelKey {
    int x, y, z;

    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const {
      // Simple hash combination
      return ((std::hash<int>()(key.x) ^
              (std::hash<int>()(key.y) << 1)) >> 1) ^
              (std::hash<int>()(key.z) << 1);
    }
  };

  // Map to store one point per voxel (using centroid)
  std::unordered_map<VoxelKey, height_map::Point3D, VoxelKeyHash> voxel_map;

  // Process each point
  for (const auto& point : cloud.points) {
    // Skip invalid points
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    // Compute voxel index
    VoxelKey key;
    key.x = static_cast<int>(std::floor(point.x / voxel_size_));
    key.y = static_cast<int>(std::floor(point.y / voxel_size_));
    key.z = static_cast<int>(std::floor(point.z / voxel_size_));

    // Store the point (last point in voxel wins, could be improved with centroid)
    voxel_map[key] = point;
  }

  // Rebuild cloud with filtered points
  cloud.clear();
  cloud.reserve(voxel_map.size());

  for (const auto& [key, point] : voxel_map) {
    cloud.push_back(point);
  }

  size_t points_after = cloud.size();

  // Update statistics
  total_points_processed_ += points_before;
  total_points_filtered_ += (points_before - points_after);
}

// Register this stage with the factory
REGISTER_STAGE(VoxelFilterStage)

} // namespace height_mapping::core