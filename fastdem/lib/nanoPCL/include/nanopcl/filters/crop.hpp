// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_CROP_HPP
#define NANOPCL_FILTERS_CROP_HPP

#include "nanopcl/filters/core.hpp"

namespace nanopcl {
namespace filters {

// Box Cropping (Axis-aligned)
[[nodiscard]] PointCloud cropBox(const PointCloud& cloud,
                                 const Point& min,
                                 const Point& max,
                                 FilterMode mode = FilterMode::INSIDE);

[[nodiscard]] PointCloud cropBox(PointCloud&& cloud, const Point& min, const Point& max, FilterMode mode = FilterMode::INSIDE);

// Range Cropping (Distance from origin)
[[nodiscard]] PointCloud cropRange(const PointCloud& cloud, float min_range, float max_range, FilterMode mode = FilterMode::INSIDE);

[[nodiscard]] PointCloud cropRange(PointCloud&& cloud, float min_range, float max_range, FilterMode mode = FilterMode::INSIDE);

// Single Axis Cropping
[[nodiscard]] PointCloud cropX(const PointCloud& cloud, float min, float max, FilterMode mode = FilterMode::INSIDE);
[[nodiscard]] PointCloud cropX(PointCloud&& cloud, float min, float max, FilterMode mode = FilterMode::INSIDE);

[[nodiscard]] PointCloud cropY(const PointCloud& cloud, float min, float max, FilterMode mode = FilterMode::INSIDE);
[[nodiscard]] PointCloud cropY(PointCloud&& cloud, float min, float max, FilterMode mode = FilterMode::INSIDE);

[[nodiscard]] PointCloud cropZ(const PointCloud& cloud, float min, float max, FilterMode mode = FilterMode::INSIDE);
[[nodiscard]] PointCloud cropZ(PointCloud&& cloud, float min, float max, FilterMode mode = FilterMode::INSIDE);

// Angle Cropping (Azimuth / Horizontal FOV)
[[nodiscard]] PointCloud cropAngle(const PointCloud& cloud, float min_angle, float max_angle, FilterMode mode = FilterMode::INSIDE);
[[nodiscard]] PointCloud cropAngle(PointCloud&& cloud, float min_angle, float max_angle, FilterMode mode = FilterMode::INSIDE);

} // namespace filters
} // namespace nanopcl

#include "nanopcl/filters/impl/crop_impl.hpp"

#endif // NANOPCL_FILTERS_CROP_HPP
