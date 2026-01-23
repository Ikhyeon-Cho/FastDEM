// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_CROP_HPP
#define NANOPCL_FILTERS_CROP_HPP

#include "nanopcl/filters/core.hpp"

namespace npcl {
namespace filters {

// =============================================================================
// Box Cropping (Axis-aligned Bounding Box)
// =============================================================================

/**
 * @brief Crop points by axis-aligned bounding box (AABB)
 *
 * Keeps or removes points based on whether their XYZ coordinates are within
 * the box defined by [min, max].
 *
 * @param cloud Input point cloud
 * @param min Minimum corner (x_min, y_min, z_min)
 * @param max Maximum corner (x_max, y_max, z_max)
 * @param mode Filter mode (default: INSIDE)
 * @return New point cloud with filtered points
 *
 * @code
 *   Point min(-10, -10, -2), max(10, 10, 2);
 *   auto filtered = filters::cropBox(cloud, min, max);
 * @endcode
 */
[[nodiscard]] PointCloud cropBox(const PointCloud& cloud, const Point& min,
                                 const Point& max,
                                 FilterMode mode = FilterMode::INSIDE);

/**
 * @brief Crop points by axis-aligned bounding box (move version)
 */
[[nodiscard]] PointCloud cropBox(PointCloud&& cloud, const Point& min,
                                 const Point& max,
                                 FilterMode mode = FilterMode::INSIDE);

// =============================================================================
// Range Cropping (Distance from center)
// =============================================================================

/**
 * @brief Crop points by Euclidean distance from a center point
 *
 * @param cloud Input point cloud
 * @param min_range Minimum distance (meters, inclusive)
 * @param max_range Maximum distance (meters, inclusive)
 * @param center Center point for distance calculation (default: origin)
 * @param mode Filter mode
 * @return New point cloud with filtered points
 */
[[nodiscard]] PointCloud cropRange(const PointCloud& cloud, float min_range,
                                   float max_range,
                                   const Point& center = Point::Zero(),
                                   FilterMode mode = FilterMode::INSIDE);

/**
 * @brief Crop points by Euclidean distance (move version)
 */
[[nodiscard]] PointCloud cropRange(PointCloud&& cloud, float min_range,
                                   float max_range,
                                   const Point& center = Point::Zero(),
                                   FilterMode mode = FilterMode::INSIDE);

/**
 * @brief Crop points by range with mode (center defaults to origin) (copy)
 */
[[nodiscard]] PointCloud cropRange(const PointCloud& cloud, float min_range,
                                   float max_range, FilterMode mode);

/**
 * @brief Crop points by range with mode (center defaults to origin) (move)
 */
[[nodiscard]] PointCloud cropRange(PointCloud&& cloud, float min_range,
                                   float max_range, FilterMode mode);

// =============================================================================
// Single Axis Cropping (Passthrough)
// =============================================================================

/**
 * @brief Crop points by X axis range (copy version)
 *
 * @param cloud Input point cloud (unchanged)
 * @param min Minimum X value (inclusive)
 * @param max Maximum X value (inclusive)
 * @param mode INSIDE: keep points in range, OUTSIDE: keep points outside
 * @return New point cloud containing filtered points
 */
[[nodiscard]] PointCloud cropXaxis(const PointCloud& cloud, float min, float max,
                               FilterMode mode = FilterMode::INSIDE);

/// @brief Crop points by X axis range (move version)
[[nodiscard]] PointCloud cropXaxis(PointCloud&& cloud, float min, float max,
                               FilterMode mode = FilterMode::INSIDE);

/**
 * @brief Crop points by Y axis range (copy version)
 *
 * @param cloud Input point cloud (unchanged)
 * @param min Minimum Y value (inclusive)
 * @param max Maximum Y value (inclusive)
 * @param mode INSIDE: keep points in range, OUTSIDE: keep points outside
 * @return New point cloud containing filtered points
 */
[[nodiscard]] PointCloud cropYaxis(const PointCloud& cloud, float min, float max,
                               FilterMode mode = FilterMode::INSIDE);

/// @brief Crop points by Y axis range (move version)
[[nodiscard]] PointCloud cropYaxis(PointCloud&& cloud, float min, float max,
                               FilterMode mode = FilterMode::INSIDE);

/**
 * @brief Crop points by Z axis range (copy version)
 *
 * @param cloud Input point cloud (unchanged)
 * @param min Minimum Z value (inclusive)
 * @param max Maximum Z value (inclusive)
 * @param mode INSIDE: keep points in range, OUTSIDE: keep points outside
 * @return New point cloud containing filtered points
 */
[[nodiscard]] PointCloud cropZaxis(const PointCloud& cloud, float min, float max,
                               FilterMode mode = FilterMode::INSIDE);

/// @brief Crop points by Z axis range (move version)
[[nodiscard]] PointCloud cropZaxis(PointCloud&& cloud, float min, float max,
                               FilterMode mode = FilterMode::INSIDE);

// =============================================================================
// Angle Cropping (Azimuth/Horizontal FOV)
// =============================================================================

/**
 * @brief Crop points by azimuth angle (horizontal Field of View)
 *
 * Filters points based on their horizontal yaw angle in the range [-PI, PI].
 *
 * @param cloud Input point cloud
 * @param min_angle Start angle in radians (inclusive)
 * @param max_angle End angle in radians (inclusive)
 * @param mode Filter mode
 * @return New point cloud containing filtered points
 *
 * @note **Wrap-around Support**: If `min_angle > max_angle`, the filter automatically
 *       crops the sector that crosses the -PI/PI boundary.
 *       Example: `min=3.0, max=-3.0` keeps a small sector (approx. 16 degrees) 
 *       centered at the back.
 *
 * @note **Performance**: Implements a cross-product based approach to determine
 *       angular orientation, completely avoiding expensive `atan2()` calls.
 *       Benchmarked to be ~56x faster than naive implementations.
 */
[[nodiscard]] PointCloud cropAngle(const PointCloud& cloud, float min_angle,
                                   float max_angle,
                                   FilterMode mode = FilterMode::INSIDE);

/**
 * @brief Crop points by azimuth angle (move version)
 */
[[nodiscard]] PointCloud cropAngle(PointCloud&& cloud, float min_angle,
                                   float max_angle,
                                   FilterMode mode = FilterMode::INSIDE);

}  // namespace filters
}  // namespace npcl

#include "nanopcl/filters/impl/crop_impl.hpp"

#endif  // NANOPCL_FILTERS_CROP_HPP
