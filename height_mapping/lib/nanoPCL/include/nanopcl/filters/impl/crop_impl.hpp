// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for crop filters.
// Do not include this file directly; include <nanopcl/filters/crop.hpp>

#ifndef NANOPCL_FILTERS_IMPL_CROP_IMPL_HPP
#define NANOPCL_FILTERS_IMPL_CROP_IMPL_HPP

#include <cmath>
#include <utility>

namespace npcl {
namespace filters {

// =============================================================================
// Spatial Filters: cropBox
// =============================================================================
inline PointCloud cropBox(const PointCloud& cloud, const Point& min,
                          const Point& max, FilterMode mode) {
  if (mode == FilterMode::INSIDE) {
    return detail::filterByPointCopy(cloud, [&](const Point& pt) {
      return pt.x() >= min.x() && pt.x() <= max.x() && pt.y() >= min.y() &&
             pt.y() <= max.y() && pt.z() >= min.z() && pt.z() <= max.z();
    });
  } else {
    return detail::filterByPointCopy(cloud, [&](const Point& pt) {
      return pt.x() < min.x() || pt.x() > max.x() || pt.y() < min.y() ||
             pt.y() > max.y() || pt.z() < min.z() || pt.z() > max.z();
    });
  }
}

inline PointCloud cropBox(PointCloud&& cloud, const Point& min,
                          const Point& max, FilterMode mode) {
  if (mode == FilterMode::INSIDE) {
    detail::filterByPointInPlace(cloud, [&](const Point& pt) {
      return pt.x() >= min.x() && pt.x() <= max.x() && pt.y() >= min.y() &&
             pt.y() <= max.y() && pt.z() >= min.z() && pt.z() <= max.z();
    });
  } else {
    detail::filterByPointInPlace(cloud, [&](const Point& pt) {
      return pt.x() < min.x() || pt.x() > max.x() || pt.y() < min.y() ||
             pt.y() > max.y() || pt.z() < min.z() || pt.z() > max.z();
    });
  }
  return std::move(cloud);
}

// =============================================================================
// Spatial Filters: cropRange
// =============================================================================
inline PointCloud cropRange(const PointCloud& cloud, float min_range,
                            float max_range, const Point& center,
                            FilterMode mode) {
  const float min_sq = min_range * min_range;
  const float max_sq = max_range * max_range;

  if (mode == FilterMode::INSIDE) {
    return detail::filterByPointCopy(cloud, [&](const Point& pt) {
      float dist_sq = (pt - center).squaredNorm();
      return dist_sq >= min_sq && dist_sq <= max_sq;
    });
  } else {
    return detail::filterByPointCopy(cloud, [&](const Point& pt) {
      float dist_sq = (pt - center).squaredNorm();
      return dist_sq < min_sq || dist_sq > max_sq;
    });
  }
}

inline PointCloud cropRange(PointCloud&& cloud, float min_range,
                            float max_range, const Point& center,
                            FilterMode mode) {
  const float min_sq = min_range * min_range;
  const float max_sq = max_range * max_range;

  if (mode == FilterMode::INSIDE) {
    detail::filterByPointInPlace(cloud, [&](const Point& pt) {
      float dist_sq = (pt - center).squaredNorm();
      return dist_sq >= min_sq && dist_sq <= max_sq;
    });
  } else {
    detail::filterByPointInPlace(cloud, [&](const Point& pt) {
      float dist_sq = (pt - center).squaredNorm();
      return dist_sq < min_sq || dist_sq > max_sq;
    });
  }
  return std::move(cloud);
}

inline PointCloud cropRange(const PointCloud& cloud, float min_range,
                            float max_range, FilterMode mode) {
  return cropRange(cloud, min_range, max_range, Point::Zero(), mode);
}

inline PointCloud cropRange(PointCloud&& cloud, float min_range,
                            float max_range, FilterMode mode) {
  return cropRange(std::move(cloud), min_range, max_range, Point::Zero(), mode);
}

// =============================================================================
// Axis Filters: cropXaxis, cropYaxis, cropZaxis
// =============================================================================
namespace detail {

template <int Axis>
inline float getAxis(const Point& pt) {
  if constexpr (Axis == 0)
    return pt.x();
  else if constexpr (Axis == 1)
    return pt.y();
  else
    return pt.z();
}

template <int Axis>
inline PointCloud cropAxisCopy(const PointCloud& cloud, float min, float max,
                               FilterMode mode) {
  if (mode == FilterMode::INSIDE) {
    return filterByPointCopy(cloud, [=](const Point& pt) {
      float v = getAxis<Axis>(pt);
      return v >= min && v <= max;
    });
  } else {
    return filterByPointCopy(cloud, [=](const Point& pt) {
      float v = getAxis<Axis>(pt);
      return v < min || v > max;
    });
  }
}

template <int Axis>
inline PointCloud cropAxisMove(PointCloud&& cloud, float min, float max,
                               FilterMode mode) {
  if (mode == FilterMode::INSIDE) {
    filterByPointInPlace(cloud, [=](const Point& pt) {
      float v = getAxis<Axis>(pt);
      return v >= min && v <= max;
    });
  } else {
    filterByPointInPlace(cloud, [=](const Point& pt) {
      float v = getAxis<Axis>(pt);
      return v < min || v > max;
    });
  }
  return std::move(cloud);
}

}  // namespace detail

inline PointCloud cropXaxis(const PointCloud& cloud, float min, float max,
                        FilterMode mode) {
  return detail::cropAxisCopy<0>(cloud, min, max, mode);
}

inline PointCloud cropXaxis(PointCloud&& cloud, float min, float max,
                        FilterMode mode) {
  return detail::cropAxisMove<0>(std::move(cloud), min, max, mode);
}

inline PointCloud cropYaxis(const PointCloud& cloud, float min, float max,
                        FilterMode mode) {
  return detail::cropAxisCopy<1>(cloud, min, max, mode);
}

inline PointCloud cropYaxis(PointCloud&& cloud, float min, float max,
                        FilterMode mode) {
  return detail::cropAxisMove<1>(std::move(cloud), min, max, mode);
}

inline PointCloud cropZaxis(const PointCloud& cloud, float min, float max,
                        FilterMode mode) {
  return detail::cropAxisCopy<2>(cloud, min, max, mode);
}

inline PointCloud cropZaxis(PointCloud&& cloud, float min, float max,
                        FilterMode mode) {
  return detail::cropAxisMove<2>(std::move(cloud), min, max, mode);
}

// =============================================================================
// Angle Filter: cropAngle (azimuth)
// Supports wrap-around: cropAngle(cloud, 3π/4, -3π/4) keeps rear 90°
// Uses cross-product for ~56x faster performance than atan2
// =============================================================================
inline PointCloud cropAngle(const PointCloud& cloud, float min_angle,
                            float max_angle, FilterMode mode) {
  const float cos_min = std::cos(min_angle), sin_min = std::sin(min_angle);
  const float cos_max = std::cos(max_angle), sin_max = std::sin(max_angle);

  const bool wrap = min_angle > max_angle;
  const float range =
      wrap ? (2.0f * static_cast<float>(M_PI) - (min_angle - max_angle))
           : (max_angle - min_angle);
  const bool use_and = range < static_cast<float>(M_PI);
  const bool keep_inside = (mode == FilterMode::INSIDE);

  constexpr float eps = 1e-5f;

  return detail::filterByPointCopy(cloud, [=](const Point& pt) {
    float c_min = cos_min * pt.y() - sin_min * pt.x();
    float c_max = cos_max * pt.y() - sin_max * pt.x();
    bool in_range = use_and ? (c_min >= -eps && c_max <= eps)
                            : (c_min >= -eps || c_max <= eps);
    return in_range == keep_inside;
  });
}

inline PointCloud cropAngle(PointCloud&& cloud, float min_angle,
                            float max_angle, FilterMode mode) {
  // Precompute direction vectors
  const float cos_min = std::cos(min_angle), sin_min = std::sin(min_angle);
  const float cos_max = std::cos(max_angle), sin_max = std::sin(max_angle);

  // Determine AND/OR based on angular range size
  const bool wrap = min_angle > max_angle;
  const float range =
      wrap ? (2.0f * static_cast<float>(M_PI) - (min_angle - max_angle))
           : (max_angle - min_angle);
  const bool use_and = range < static_cast<float>(M_PI);
  const bool keep_inside = (mode == FilterMode::INSIDE);

  // Epsilon for boundary tolerance (handles float precision at exact
  // boundaries)
  constexpr float eps = 1e-5f;

  detail::filterByPointInPlace(cloud, [=](const Point& pt) {
    // Cross products: determines which side of each boundary vector
    float c_min = cos_min * pt.y() - sin_min * pt.x();
    float c_max = cos_max * pt.y() - sin_max * pt.x();
    bool in_range = use_and ? (c_min >= -eps && c_max <= eps)
                            : (c_min >= -eps || c_max <= eps);
    return in_range == keep_inside;
  });

  return std::move(cloud);
}

}  // namespace filters
}  // namespace npcl

#endif  // NANOPCL_FILTERS_IMPL_CROP_IMPL_HPP
