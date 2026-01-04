// nanopcl/filters/downsample.hpp
// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_DOWNSAMPLE_HPP
#define NANOPCL_FILTERS_DOWNSAMPLE_HPP

#include <cstdint>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {
namespace filters {

// =============================================================================
// Voxel Grid Downsampling
// =============================================================================

/// Voxel reduction method
enum class VoxelMethod {
  CENTROID,     ///< Average position of all points in voxel (default)
  FIRST,        ///< First point added to voxel
  ARBITRARY,    ///< Deterministic arbitrary selection (reproducible)
  CLOSEST,      ///< Point closest to voxel center
  VOXEL_CENTER  ///< Voxel center itself
};

/**
 * @brief Downsample point cloud using voxel grid filter
 *
 * @param cloud Input point cloud
 * @param voxel_size Size of voxel cube in meters
 * @return Downsampled point cloud
 *
 * @throws std::invalid_argument if voxel_size is not in (0, 100]
 */
[[nodiscard]] PointCloud voxelGrid(const PointCloud& cloud, float voxel_size);

/**
 * @brief Downsample point cloud using voxel grid filter with specified method
 *
 * @param cloud Input point cloud
 * @param voxel_size Size of voxel cube in meters
 * @param method Reduction method for selecting representative point
 * @return Downsampled point cloud
 *
 * @throws std::invalid_argument if voxel_size is not in (0, 100]
 */
[[nodiscard]] PointCloud voxelGrid(const PointCloud& cloud, float voxel_size,
                                   VoxelMethod method);

/**
 * @brief Downsample point cloud using voxel grid filter (move version)
 *
 * @param cloud Input point cloud (consumed, do not use after call)
 * @param voxel_size Size of voxel cube in meters
 * @return Downsampled point cloud (reuses input memory)
 *
 * @note This version reuses the input cloud's memory buffer for optimal
 *       performance. Use std::move() to enable this optimization.
 *       Benchmarked: ~1.4x faster than copy version.
 *
 * @throws std::invalid_argument if voxel_size is not in (0, 100]
 */
[[nodiscard]] PointCloud voxelGrid(PointCloud&& cloud, float voxel_size);

/**
 * @brief Downsample point cloud using voxel grid filter (move version)
 */
[[nodiscard]] PointCloud voxelGrid(PointCloud&& cloud, float voxel_size,
                                   VoxelMethod method);

}  // namespace filters
}  // namespace nanopcl

// Include implementation
#include "nanopcl/filters/impl/voxel_grid_impl.hpp"

#endif  // NANOPCL_FILTERS_DOWNSAMPLE_HPP
