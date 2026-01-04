// nanopcl/filters/core.hpp
// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_CORE_HPP
#define NANOPCL_FILTERS_CORE_HPP

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {
namespace filters {

/// Filter mode for spatial filters
enum class FilterMode {
  INSIDE,  ///< Keep points inside the region (default)
  OUTSIDE  ///< Keep points outside the region
};

// =============================================================================
// Generic Filter
// =============================================================================

/**
 * @brief Filter point cloud with user-defined predicate (copy version)
 *
 * @tparam Predicate Callable with signature: bool(ConstPointRef)
 * @param cloud Input point cloud (unchanged)
 * @param predicate Returns true for points to KEEP
 * @return New point cloud containing only points that pass the predicate
 *
 * Example:
 * @code
 * auto filtered = filters::filter(cloud, [](auto p) {
 *   return p.point().z() > 0 && p.hasIntensity() && p.intensity() > 0.5f;
 * });
 * @endcode
 */
template <typename Predicate>
[[nodiscard]] PointCloud filter(const PointCloud& cloud, Predicate predicate);

/**
 * @brief Filter point cloud with user-defined predicate (move version)
 *
 * @tparam Predicate Callable with signature: bool(ConstPointRef)
 * @param cloud Input point cloud (consumed, do not use after call)
 * @param predicate Returns true for points to KEEP
 * @return Filtered point cloud (reuses input memory)
 *
 * @note This version reuses the input cloud's memory buffer for optimal
 *       performance. Use std::move() to enable this optimization.
 *
 * Example:
 * @code
 * cloud = filters::filter(std::move(cloud), [](auto p) {
 *   return p.point().z() > 0;
 * });
 * @endcode
 */
template <typename Predicate>
[[nodiscard]] PointCloud filter(PointCloud&& cloud, Predicate predicate);

// =============================================================================
// Utility Filters
// =============================================================================

/**
 * @brief Remove points with invalid coordinates (NaN or Inf) (copy version)
 *
 * @param cloud Input point cloud (unchanged)
 * @return New point cloud containing only valid (finite) points
 */
[[nodiscard]] PointCloud removeInvalid(const PointCloud& cloud);

/**
 * @brief Remove points with invalid coordinates (NaN or Inf) (move version)
 */
[[nodiscard]] PointCloud removeInvalid(PointCloud&& cloud);

}  // namespace filters
}  // namespace nanopcl

#include "nanopcl/filters/impl/core_impl.hpp"

#endif  // NANOPCL_FILTERS_CORE_HPP
