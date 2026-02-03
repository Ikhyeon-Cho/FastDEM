// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_DOWNSAMPLE_HPP
#define NANOPCL_FILTERS_DOWNSAMPLE_HPP

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {
namespace filters {

/// Voxel reduction mode
enum class VoxelMode {
  CENTROID, ///< Average all points (Best quality, recommended)
  NEAREST,  ///< Point closest to voxel center (Preserves original values)
  ANY,      ///< Deterministic selection (Fastest)
  CENTER    ///< Voxel center coordinate (Artificial grid).
            ///< @warning May cause layer separation at voxel boundaries when
            ///< input has noise (e.g., z≈0 with ±noise splits into z=±voxel/2).
            ///< Use CENTROID for general downsampling.
};

// Voxel Grid Downsampling
[[nodiscard]] PointCloud voxelGrid(const PointCloud& cloud, float voxel_size, VoxelMode mode = VoxelMode::CENTROID);
[[nodiscard]] PointCloud voxelGrid(PointCloud&& cloud, float voxel_size, VoxelMode mode = VoxelMode::CENTROID);

// 2D Grid Max-Z Selection (for height map edge preservation)
[[nodiscard]] PointCloud gridMaxZ(const PointCloud& cloud, float grid_size);
[[nodiscard]] PointCloud gridMaxZ(PointCloud&& cloud, float grid_size);

} // namespace filters
} // namespace nanopcl

#include "nanopcl/filters/impl/grid_max_z_impl.hpp"
#include "nanopcl/filters/impl/voxel_grid_impl.hpp"

#endif // NANOPCL_FILTERS_DOWNSAMPLE_HPP
