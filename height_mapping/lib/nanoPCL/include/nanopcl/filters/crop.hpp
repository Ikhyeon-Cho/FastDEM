// nanopcl/filters/crop.hpp
// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_CROP_HPP
#define NANOPCL_FILTERS_CROP_HPP

#include "nanopcl/filters/core.hpp"

namespace nanopcl {
namespace filters {

// =============================================================================
// Box Cropping (AABB)
// =============================================================================

/**
 * @brief Crop points by axis-aligned bounding box (copy version)
 *
 * @param cloud Input point cloud (unchanged)
 * @param min Minimum corner (x_min, y_min, z_min)
 * @param max Maximum corner (x_max, y_max, z_max)
 * @param mode INSIDE: keep points in box, OUTSIDE: keep points outside box
 * @return New point cloud containing only points in/outside the box
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
 * @brief Crop points by range (distance) from a center point (copy version)
 *
 * @param cloud Input point cloud (unchanged)
 * @param min_range Minimum range (inclusive)
 * @param max_range Maximum range (inclusive)
 * @param center Center point for range calculation (default: origin)
 * @param mode INSIDE: keep points in range, OUTSIDE: keep points outside range
 * @return New point cloud containing only points in/outside the range
 */
[[nodiscard]] PointCloud cropRange(const PointCloud& cloud, float min_range,
                                   float max_range,
                                   const Point& center = Point::Zero(),
                                   FilterMode mode = FilterMode::INSIDE);

/**
 * @brief Crop points by range (distance) from a center point (move version)
 */
[[nodiscard]] PointCloud cropRange(PointCloud&& cloud, float min_range,
                                   float max_range,
                                   const Point& center = Point::Zero(),
                                   FilterMode mode = FilterMode::INSIDE);

// =============================================================================
// Axis Cropping (Passthrough)
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
[[nodiscard]] PointCloud cropX(const PointCloud& cloud, float min, float max,
                               FilterMode mode = FilterMode::INSIDE);

/// @brief Crop points by X axis range (move version)
[[nodiscard]] PointCloud cropX(PointCloud&& cloud, float min, float max,
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
[[nodiscard]] PointCloud cropY(const PointCloud& cloud, float min, float max,
                               FilterMode mode = FilterMode::INSIDE);

/// @brief Crop points by Y axis range (move version)
[[nodiscard]] PointCloud cropY(PointCloud&& cloud, float min, float max,
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
[[nodiscard]] PointCloud cropZ(const PointCloud& cloud, float min, float max,
                               FilterMode mode = FilterMode::INSIDE);

/// @brief Crop points by Z axis range (move version)
[[nodiscard]] PointCloud cropZ(PointCloud&& cloud, float min, float max,
                               FilterMode mode = FilterMode::INSIDE);

// =============================================================================
// Angle Cropping (Azimuth/Horizontal FOV)
// =============================================================================

/**
 * @brief Crop points by azimuth angle (horizontal FOV) (copy version)
 *
 * @param cloud Input point cloud (unchanged)
 * @param min_angle Minimum angle in radians (inclusive)
 * @param max_angle Maximum angle in radians (inclusive)
 * @param mode INSIDE: keep points in range, OUTSIDE: keep points outside
 * @return New point cloud containing filtered points
 *
 * @note Angle is computed as atan2(y, x), range [-PI, PI]
 *       Supports wrap-around: cropAngle(cloud, 3π/4, -3π/4) keeps rear 90°
 *       Uses cross-product for ~56x faster performance than atan2
 */
[[nodiscard]] PointCloud cropAngle(const PointCloud& cloud, float min_angle,
                                   float max_angle,
                                   FilterMode mode = FilterMode::INSIDE);

/// @brief Crop points by azimuth angle (horizontal FOV) (move version)
[[nodiscard]] PointCloud cropAngle(PointCloud&& cloud, float min_angle,
                                   float max_angle,
                                   FilterMode mode = FilterMode::INSIDE);

}  // namespace filters
}  // namespace nanopcl

#include "nanopcl/filters/impl/crop_impl.hpp"

#endif  // NANOPCL_FILTERS_CROP_HPP
