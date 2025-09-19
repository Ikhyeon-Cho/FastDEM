/*
 * voxel_filter_stage.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_STAGES_VOXEL_FILTER_STAGE_H
#define HEIGHT_MAPPING_CORE_PIPELINE_STAGES_VOXEL_FILTER_STAGE_H

#include "height_mapping_core/pipeline/processing_context.h"
#include "height_mapping_core/pipeline/stage.h"
#include <unordered_map>

namespace height_mapping::core {

// Stage that performs voxel grid filtering
class VoxelFilterStage : public PipelineStage<VoxelFilterStage> {
public:
  static constexpr const char *STAGE_NAME = "VoxelFilter";
  static constexpr const char *STAGE_TYPE = "Point Cloud Filter";

  // Constructor
  explicit VoxelFilterStage(float voxel_size = 0.05f)
      : voxel_size_(voxel_size) {}

  void processImpl(ProcessingContext &ctx) {
    if (ctx.cloud().empty())
      return;

    size_t points_before = ctx.cloud().size();
    // std::cout << "[VoxelFilter] Input: " << points_before << " points" <<
    // std::endl;

    // Perform voxel filtering in-place (much faster)
    voxelFilterInPlace(ctx.cloud(), voxel_size_);

    size_t points_after = ctx.cloud().size();
    // std::cout << "[VoxelFilter] Output: " << points_after << " points"
    //           << " (voxel size: " << voxel_size_ << " m)" << std::endl;

    // if (points_after == 0) {
    //     std::cout << "[VoxelFilter] WARNING: All points filtered out!" <<
    //     std::endl;
    // }

    // Update statistics
    ctx.stats().points_processed = points_before;
    ctx.stats().points_filtered = points_before - points_after;

    // Store metadata
    ctx.set("voxel_size", voxel_size_);
    ctx.set("filter_reduction_ratio",
              static_cast<float>(points_after) / points_before);
  }

  bool canProcess(const ProcessingContext &ctx) const override {
    return !frame.cloud().empty() && voxel_size_ > 0;
  }

  // Configuration
  void setVoxelSize(float size) {
    if (size > 0) {
      voxel_size_ = size;
    }
  }

  float getVoxelSize() const { return voxel_size_; }

private:
  // Voxel key for spatial hashing
  struct VoxelKey {
    int x, y, z;

    bool operator==(const VoxelKey &other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey &key) const {
      // Simple hash combination
      std::size_t h1 = std::hash<int>()(key.x);
      std::size_t h2 = std::hash<int>()(key.y);
      std::size_t h3 = std::hash<int>()(key.z);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

  VoxelKey getVoxelKey(const Point3D &point, float voxel_size) const {
    return VoxelKey{static_cast<int>(std::floor(point.x / voxel_size)),
                    static_cast<int>(std::floor(point.y / voxel_size)),
                    static_cast<int>(std::floor(point.z / voxel_size))};
  }

  void voxelFilterInPlace(PointCloudXYZ &cloud, float voxel_size) const {
    if (cloud.empty())
      return;

    // Use a simpler hash map with pre-allocated size
    const size_t estimated_voxels = cloud.size() / 2; // Estimate ~50% reduction
    std::unordered_map<VoxelKey, size_t, VoxelKeyHash> voxel_indices;
    voxel_indices.reserve(estimated_voxels);

    size_t write_idx = 0;

    // Single pass: keep highest point per voxel
    for (size_t i = 0; i < cloud.points.size(); ++i) {
      const auto &point = cloud.points[i];
      if (!point.isFinite())
        continue;

      VoxelKey key = getVoxelKey(point, voxel_size);

      auto it = voxel_indices.find(key);
      if (it == voxel_indices.end()) {
        // First point in this voxel - keep it
        voxel_indices[key] = write_idx;
        if (write_idx != i) {
          cloud.points[write_idx] = point;
        }
        write_idx++;
      } else {
        // Check if this point is higher
        if (point.z > cloud.points[it->second].z) {
          cloud.points[it->second] = point;
        }
      }
    }

    // Resize to keep only filtered points
    cloud.points.resize(write_idx);
  }

  float voxel_size_ = 0.05f; // Default 5cm voxels
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_STAGES_VOXEL_FILTER_STAGE_H