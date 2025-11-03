/*
 * voxel_filter.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ALGORITHMS_VOXEL_FILTER_H
#define HEIGHT_MAPPING_CORE_ALGORITHMS_VOXEL_FILTER_H

#include <map>
#include <tuple>
#include <vector>

#include "height_mapping/core/data/point_cloud.h"

namespace height_mapping::algorithms {

/**
 * @brief Voxel-based downsampling filter algorithm
 *
 * Reduces point cloud density by grouping points into voxels and
 * selecting a representative point for each voxel.
 */
class VoxelFilter {
 public:
  enum class ReductionMethod {
    CENTROID,     // Average of all points in voxel
    RANDOM,       // Random point from voxel
    FIRST,        // First point added to voxel
    CLOSEST,      // Point closest to voxel center
    VOXEL_CENTER  // Voxel center itself
  };

  /**
   * @brief Configuration parameters for voxel filter
   */
  struct Config {
    float voxel_size = 0.1f;
    ReductionMethod reduction_method = ReductionMethod::CENTROID;

    void validate() const {
      if (voxel_size <= 0 || voxel_size > 10.0f) {
        throw std::invalid_argument("voxel_size must be in (0, 10]");
      }
    }
  };

  /**
   * @brief Statistics from filtering operation
   */
  struct Stats {
    size_t original_size = 0;
    size_t filtered_size = 0;
    size_t voxel_count = 0;
  };

 private:
  Config config_;
  using VoxelKey = std::tuple<int, int, int>;

  struct VoxelData {
    std::vector<Point3D> points;
    std::vector<size_t> indices;  // Original indices for sensor data
  };

 public:
  VoxelFilter() : config_{} { config_.validate(); }

  explicit VoxelFilter(const Config &config) : config_(config) {
    config_.validate();
  }

  void setConfig(const Config &config) {
    config_ = config;
    config_.validate();
  }

  const Config &getConfig() const { return config_; }

  /**
   * @brief Apply voxel filter in-place
   * @param cloud Point cloud to filter (modified in-place)
   * @return Statistics from the filtering operation
   */
  Stats filterInPlace(PointCloud &cloud) const;

  /**
   * @brief Apply voxel filter to copy of cloud
   * @param cloud Input point cloud (not modified)
   * @param stats Statistics from the filtering operation
   * @return Filtered copy of the cloud
   */
  PointCloud filter(const PointCloud &cloud, Stats &stats) const;

 private:
  VoxelKey getVoxelKey(const Point3D &point) const;

  Point3D computeRepresentativePoint(const VoxelData &voxel_data,
                                     const VoxelKey &key) const;

  void addSensorData(PointCloud &filtered_cloud,
                     const PointCloud &original_cloud,
                     const VoxelData &voxel_data,
                     const Point3D &repr_point) const;
};

}  // namespace height_mapping::algorithms

#endif  // HEIGHT_MAPPING_CORE_ALGORITHMS_VOXEL_FILTER_H