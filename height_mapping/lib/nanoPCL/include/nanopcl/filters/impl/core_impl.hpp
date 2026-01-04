// nanopcl/filters/impl/core_impl.hpp
// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for core filters.
// Do not include this file directly; include <nanopcl/filters/core.hpp>

#ifndef NANOPCL_FILTERS_IMPL_CORE_IMPL_HPP
#define NANOPCL_FILTERS_IMPL_CORE_IMPL_HPP

#include <cmath>
#include <utility>

namespace nanopcl {
namespace filters {
namespace detail {

// =============================================================================
// Internal helper: In-place filter with ConstPointRef predicate
// =============================================================================
template <typename Predicate>
inline void filterInPlace(PointCloud& cloud, Predicate predicate) {
  if (cloud.empty()) return;

  const size_t n = cloud.size();

  // Pointer caching for performance (benchmarked: 1.8x faster)
  Point* pts = cloud.xyz().data();
  float* intensity = cloud.hasIntensity() ? cloud.intensity().data() : nullptr;
  float* time = cloud.hasTime() ? cloud.time().data() : nullptr;
  uint16_t* ring = cloud.hasRing() ? cloud.ring().data() : nullptr;
  Color* color = cloud.hasColor() ? cloud.color().data() : nullptr;
  Label* label = cloud.hasLabel() ? cloud.label().data() : nullptr;
  Eigen::Vector3f* normal = cloud.hasNormal() ? cloud.normal().data() : nullptr;

  size_t write = 0;
  for (size_t read = 0; read < n; ++read) {
    if (predicate(cloud.point(read))) {
      if (write != read) {
        pts[write] = pts[read];
        if (intensity) intensity[write] = intensity[read];
        if (time) time[write] = time[read];
        if (ring) ring[write] = ring[read];
        if (color) color[write] = color[read];
        if (label) label[write] = label[read];
        if (normal) normal[write] = normal[read];
      }
      ++write;
    }
  }

  cloud.resize(write);
}

// =============================================================================
// Internal helper: In-place filter with Point& predicate (no ConstPointRef overhead)
// =============================================================================
template <typename PointPredicate>
inline void filterByPointInPlace(PointCloud& cloud, PointPredicate pred) {
  if (cloud.empty()) return;

  const size_t n = cloud.size();
  Point* pts = cloud.xyz().data();
  float* intensity = cloud.hasIntensity() ? cloud.intensity().data() : nullptr;
  float* time = cloud.hasTime() ? cloud.time().data() : nullptr;
  uint16_t* ring = cloud.hasRing() ? cloud.ring().data() : nullptr;
  Color* color = cloud.hasColor() ? cloud.color().data() : nullptr;
  Label* label = cloud.hasLabel() ? cloud.label().data() : nullptr;
  Eigen::Vector3f* normal = cloud.hasNormal() ? cloud.normal().data() : nullptr;

  size_t write = 0;
  for (size_t read = 0; read < n; ++read) {
    if (pred(pts[read])) {
      if (write != read) {
        pts[write] = pts[read];
        if (intensity) intensity[write] = intensity[read];
        if (time) time[write] = time[read];
        if (ring) ring[write] = ring[read];
        if (color) color[write] = color[read];
        if (label) label[write] = label[read];
        if (normal) normal[write] = normal[read];
      }
      ++write;
    }
  }
  cloud.resize(write);
}

// =============================================================================
// Internal helper: Copy filter with Point& predicate
// =============================================================================
template <typename PointPredicate>
inline PointCloud filterByPointCopy(const PointCloud& cloud,
                                    PointPredicate pred) {
  PointCloud result(cloud.frameId());
  result.setTimestamp(cloud.timestamp());
  result.reserve(cloud.size());

  if (cloud.hasIntensity()) result.enableIntensity();
  if (cloud.hasTime()) result.enableTime();
  if (cloud.hasRing()) result.enableRing();
  if (cloud.hasColor()) result.enableColor();
  if (cloud.hasLabel()) result.enableLabel();
  if (cloud.hasNormal()) result.enableNormal();

  const Point* pts = cloud.xyz().data();
  const float* intensity =
      cloud.hasIntensity() ? cloud.intensity().data() : nullptr;
  const float* time = cloud.hasTime() ? cloud.time().data() : nullptr;
  const uint16_t* ring = cloud.hasRing() ? cloud.ring().data() : nullptr;
  const Color* color = cloud.hasColor() ? cloud.color().data() : nullptr;
  const Label* label = cloud.hasLabel() ? cloud.label().data() : nullptr;
  const Eigen::Vector3f* normal =
      cloud.hasNormal() ? cloud.normal().data() : nullptr;

  const size_t n = cloud.size();
  for (size_t i = 0; i < n; ++i) {
    if (pred(pts[i])) {
      result.xyz().push_back(pts[i]);
      if (intensity) result.intensity().push_back(intensity[i]);
      if (time) result.time().push_back(time[i]);
      if (ring) result.ring().push_back(ring[i]);
      if (color) result.color().push_back(color[i]);
      if (label) result.label().push_back(label[i]);
      if (normal) result.normal().push_back(normal[i]);
    }
  }
  return result;
}

}  // namespace detail

// =============================================================================
// Generic Filter: Move version (optimal performance)
// =============================================================================
template <typename Predicate>
inline PointCloud filter(PointCloud&& cloud, Predicate predicate) {
  detail::filterInPlace(cloud, predicate);
  return std::move(cloud);
}

// =============================================================================
// Generic Filter: Copy version (preserves original)
// =============================================================================
template <typename Predicate>
inline PointCloud filter(const PointCloud& cloud, Predicate predicate) {
  PointCloud result(cloud.frameId());
  result.setTimestamp(cloud.timestamp());
  result.reserve(cloud.size());

  // Enable same channels as input
  if (cloud.hasIntensity()) result.enableIntensity();
  if (cloud.hasTime()) result.enableTime();
  if (cloud.hasRing()) result.enableRing();
  if (cloud.hasColor()) result.enableColor();
  if (cloud.hasLabel()) result.enableLabel();
  if (cloud.hasNormal()) result.enableNormal();

  // Pointer caching for performance
  const Point* pts = cloud.xyz().data();
  const float* intensity =
      cloud.hasIntensity() ? cloud.intensity().data() : nullptr;
  const float* time = cloud.hasTime() ? cloud.time().data() : nullptr;
  const uint16_t* ring = cloud.hasRing() ? cloud.ring().data() : nullptr;
  const Color* color = cloud.hasColor() ? cloud.color().data() : nullptr;
  const Label* label = cloud.hasLabel() ? cloud.label().data() : nullptr;
  const Eigen::Vector3f* normal =
      cloud.hasNormal() ? cloud.normal().data() : nullptr;

  const size_t n = cloud.size();
  for (size_t i = 0; i < n; ++i) {
    if (predicate(cloud.point(i))) {
      result.xyz().push_back(pts[i]);
      if (intensity) result.intensity().push_back(intensity[i]);
      if (time) result.time().push_back(time[i]);
      if (ring) result.ring().push_back(ring[i]);
      if (color) result.color().push_back(color[i]);
      if (label) result.label().push_back(label[i]);
      if (normal) result.normal().push_back(normal[i]);
    }
  }

  return result;
}

// =============================================================================
// Utility Filters: removeInvalid
// =============================================================================
inline PointCloud removeInvalid(PointCloud&& cloud) {
  detail::filterByPointInPlace(cloud, [](const Point& pt) {
    return std::isfinite(pt.x()) && std::isfinite(pt.y()) &&
           std::isfinite(pt.z());
  });
  return std::move(cloud);
}

inline PointCloud removeInvalid(const PointCloud& cloud) {
  return detail::filterByPointCopy(cloud, [](const Point& pt) {
    return std::isfinite(pt.x()) && std::isfinite(pt.y()) &&
           std::isfinite(pt.z());
  });
}

}  // namespace filters
}  // namespace nanopcl

#endif  // NANOPCL_FILTERS_IMPL_CORE_IMPL_HPP
