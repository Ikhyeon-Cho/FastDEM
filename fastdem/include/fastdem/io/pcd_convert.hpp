// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * pcd_convert.hpp
 *
 * Conversions between PointCloud and ElevationMap.
 * Stateless, one-shot — no sensor model, transforms, or temporal estimation.
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_IO_PCD_CONVERT_HPP
#define FASTDEM_IO_PCD_CONVERT_HPP

#include "fastdem/config/rasterization.hpp"
#include "fastdem/elevation_map.hpp"
#include "fastdem/point_types.hpp"

namespace fastdem {

// ─── High-level: full DEM pipeline ──────────────────────────────────

/// Configuration for buildDEM().
struct DEMConfig {
  float resolution = 0.1f;
  RasterMethod method = RasterMethod::Max;

  // Statistical outlier removal (nanoPCL SOR)
  int sor_k = 10;            ///< Number of nearest neighbors for SOR
  float sor_std_mul = 1.0f;  ///< Standard deviation multiplier threshold

  // Per-cell histogram filter (floating point removal)
  float height_threshold = 2.0f;  ///< Meters above ground peak to remove
  float bin_size = 0.0f;          ///< Histogram bin size; 0 = use resolution

  // Post-processing
  int inpaint_iterations = 3;  ///< Inpainting passes (0 = disabled)
};

/**
 * @brief Build a clean DEM from a merged point cloud.
 *
 * Full offline pipeline: SOR → per-cell histogram filter (floating point
 * removal) → rasterization → inpainting. Intended for static point cloud
 * maps (e.g., SLAM output).
 *
 * @param cloud Point cloud in map/world frame
 * @param config Pipeline configuration
 * @return ElevationMap with clean DEM layers
 */
ElevationMap buildDEM(const PointCloud& cloud, const DEMConfig& config = {});

// ─── Low-level: direct conversion ───────────────────────────────────

/**
 * @brief Build elevation map from a world-frame point cloud.
 *
 * Bins all points into grid cells and computes per-cell Welford statistics.
 * No sensor model, transforms, or temporal estimation — intended for static
 * point cloud maps (e.g., loaded from PCD).
 *
 * Output layers (auto-created if missing):
 *   - elevation:     Per method (Max→max_z, Min→min_z, Mean→mean_z)
 *   - elevation_min: Minimum z per cell
 *   - elevation_max: Maximum z per cell
 *   - variance:      Sample variance of z values per cell
 *   - n_points:      Number of points per cell
 *   - intensity:     Max intensity per cell (if input has intensity)
 *   - color:         Last color per cell (if input has color)
 *
 * @param cloud Point cloud in map/world frame
 * @param map Pre-sized ElevationMap (must have geometry set)
 * @param method Determines the elevation layer value
 */
void fromPointCloud(const PointCloud& cloud, ElevationMap& map,
                    RasterMethod method = RasterMethod::Max);

/**
 * @brief Build auto-sized elevation map from a world-frame point cloud.
 *
 * Map geometry is determined by the cloud's XY bounding box.
 * Produces the same output layers as the pre-sized overload.
 *
 * @param cloud Point cloud in map/world frame
 * @param resolution Grid cell size in meters
 * @param method Determines the elevation layer value
 * @return ElevationMap sized to fit the cloud
 */
ElevationMap fromPointCloud(const PointCloud& cloud, float resolution,
                            RasterMethod method = RasterMethod::Max);

/**
 * @brief Convert ElevationMap to PointCloud (cell centers with elevation as z).
 *
 * Iterates valid elevation cells and creates points at grid cell centers.
 * Preserves intensity and color if present.
 *
 * @param map ElevationMap with at least the elevation layer
 * @return PointCloud with one point per valid cell
 */
PointCloud toPointCloud(const ElevationMap& map);

}  // namespace fastdem

#endif  // FASTDEM_IO_PCD_CONVERT_HPP
