/*
 * voxel_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_VOXEL_FILTER_H
#define HEIGHT_MAPPING_CORE_STAGES_VOXEL_FILTER_H

#include "height_mapping_core/geometry/point_cloud.h"
#include "pipeline_core/stage.h"

namespace height_mapping::core::stages {

/**
 * @brief Stage that performs voxel grid filtering on point clouds
 *
 * Reduces point cloud density by dividing space into voxels and
 * keeping only one representative point per voxel.
 *
 * Supported reduction methods:
 * - CENTROID: Average position of all points in voxel
 * - RANDOM: Random point from voxel (fastest)
 * - FIRST: First point encountered in voxel
 * - CLOSEST: Point closest to voxel center
 * - VOXEL_CENTER: Voxel center position (no actual measurement)
 */
class VoxelFilter : public pipeline::Stage {
public:
  enum class ReductionMethod {
    CENTROID,    // Mean of all points (default)
    RANDOM,      // Random selection
    FIRST,       // First point encountered
    CLOSEST,     // Closest to voxel center
    VOXEL_CENTER // Use voxel center coordinates
  };

  VoxelFilter();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *        - "voxel_size": Size of voxels in meters (default: 0.05)
   *        - "reduction_method": centroid|random|first|closest|center (default:
   * centroid)
   */
  void configure(const std::map<std::string, std::string> &params) override;

  // Getters
  float getVoxelSize() const { return voxel_size_; }
  ReductionMethod getReductionMethod() const { return reduction_method_; }

protected:
  void processImpl(pipeline::Context &ctx) override;

private:
  float voxel_size_ = 0.05f; // Default 5cm voxels
  ReductionMethod reduction_method_ = ReductionMethod::CENTROID;

  // Helper struct for voxel grid
  struct VoxelKey {
    int x, y, z;
    bool operator<(const VoxelKey &other) const {
      if (x != other.x)
        return x < other.x;
      if (y != other.y)
        return y < other.y;
      return z < other.z;
    }
  };

  // Convert point to voxel key
  VoxelKey pointToVoxel(float x, float y, float z) const;

  // Reduction methods
  ::geometry::Point3D
  reduceCentroid(const std::vector<::geometry::Point3D> &points) const;
  ::geometry::Point3D
  reduceRandom(const std::vector<::geometry::Point3D> &points) const;
  ::geometry::Point3D
  reduceFirst(const std::vector<::geometry::Point3D> &points) const;
  ::geometry::Point3D
  reduceClosest(const std::vector<::geometry::Point3D> &points,
                const VoxelKey &voxel) const;
  ::geometry::Point3D reduceVoxelCenter(const VoxelKey &voxel) const;
};

} // namespace height_mapping::core::stages

#endif // HEIGHT_MAPPING_CORE_STAGES_VOXEL_FILTER_H