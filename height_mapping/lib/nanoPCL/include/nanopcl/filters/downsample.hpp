// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_DOWNSAMPLE_HPP
#define NANOPCL_FILTERS_DOWNSAMPLE_HPP

#include <cstdint>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"

namespace npcl {
namespace filters {

// =============================================================================
// Voxel Grid Downsampling
// =============================================================================

/// Voxel reduction method
enum class VoxelMethod {
  CENTROID,     ///< Average position of all points in voxel (Most accurate, slower)
  FIRST,        ///< First point added to voxel (Fastest, depends on input order)
  ARBITRARY,    ///< Deterministic arbitrary selection (Fast, reproducible)
  CLOSEST,      ///< Point closest to voxel center (Preserves original points)
  VOXEL_CENTER  ///< Voxel center coordinate (Artificial grid points)
};

/**
 * @brief Downsample point cloud using voxel grid filter
 *
 * Divides the space into a 3D grid and reduces points within each voxel
 * to a single representative point.
 *
 * @note **Spatial Limits (21-bit coordinate packing)**:
 *   Points are mapped to integer coordinates with a limited range.
 *   | voxel_size | Valid range (from origin) | Typical Use Case    |
 *   |------------|---------------------------|---------------------|
 *   | 0.1m       | ±104 km                   | Outdoor Robotics    |
 *   | 0.01m      | ±10 km                    | Urban Mapping       |
 *   | 0.001m     | ±1 km                     | Indoor/Small-scale  |
 *
 *   Points outside this range will be clamped to the boundaries, causing
 *   data distortion. For sub-millimeter precision in large areas,
 *   pre-center your data near the origin.
 *
 * @param cloud Input point cloud
 * @param voxel_size Size of voxel cube in meters [0.001, 100.0]
 * @return Downsampled point cloud
 *
 * @throws std::invalid_argument if voxel_size is out of [0.001, 100.0] range.
 */
[[nodiscard]] PointCloud voxelGrid(const PointCloud& cloud, float voxel_size);

/**
 * @brief Downsample point cloud using voxel grid filter with specified method
 *
 * @param cloud Input point cloud
 * @param voxel_size Size of voxel cube in meters
 * @param method Reduction method for selecting representative point:
 *               - CENTROID: Geometric center (default)
 *               - FIRST/ARBITRARY: Faster, keeps original points
 *               - CLOSEST: Keeps original point nearest to center
 * @return Downsampled point cloud
 *
 * @throws std::invalid_argument if voxel_size is <= 0 or > 100.0
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
}  // namespace npcl

// Include implementation
#include "nanopcl/filters/impl/voxel_grid_impl.hpp"

#endif  // NANOPCL_FILTERS_DOWNSAMPLE_HPP
