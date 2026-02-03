// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_CORE_HPP
#define NANOPCL_FILTERS_CORE_HPP

#include <cmath>
#include <utility>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {
namespace filters {

enum class FilterMode { INSIDE,
                        OUTSIDE };

namespace detail {

template <typename Predicate>
inline void filterInPlace(PointCloud& cloud, Predicate pred) {
  if (cloud.empty())
    return;

  const size_t n = cloud.size();
  const bool has_i = cloud.hasIntensity();
  const bool has_t = cloud.hasTime();
  const bool has_r = cloud.hasRing();
  const bool has_c = cloud.hasColor();
  const bool has_l = cloud.hasLabel();
  const bool has_n = cloud.hasNormal();
  const bool has_cov = cloud.hasCovariance();

  Point4* pts = cloud.points().data();
  float* intensity = has_i ? cloud.intensities().data() : nullptr;
  float* time = has_t ? cloud.times().data() : nullptr;
  uint16_t* ring = has_r ? cloud.rings().data() : nullptr;
  Color* color = has_c ? cloud.colors().data() : nullptr;
  Label* label = has_l ? cloud.labels().data() : nullptr;
  Normal4* normal = has_n ? cloud.normals().data() : nullptr;
  Covariance* cov = has_cov ? cloud.covariances().data() : nullptr;

  size_t write = 0;
  for (size_t read = 0; read < n; ++read) {
    if (pred(read)) {
      if (write != read) {
        pts[write] = pts[read];
        if (intensity)
          intensity[write] = intensity[read];
        if (time)
          time[write] = time[read];
        if (ring)
          ring[write] = ring[read];
        if (color)
          color[write] = color[read];
        if (label)
          label[write] = label[read];
        if (normal)
          normal[write] = normal[read];
        if (cov)
          cov[write] = cov[read];
      }
      ++write;
    }
  }
  cloud.resize(write);
}

template <typename Predicate>
inline PointCloud filterCopy(const PointCloud& cloud, Predicate pred) {
  std::vector<size_t> indices;
  indices.reserve(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (pred(i))
      indices.push_back(i);
  }
  return cloud.extract(indices);
}

} // namespace detail

// Generic filter: Predicate = bool(size_t index)
template <typename Predicate>
[[nodiscard]] inline PointCloud filter(PointCloud&& cloud, Predicate pred) {
  detail::filterInPlace(cloud, pred);
  return std::move(cloud);
}

template <typename Predicate>
[[nodiscard]] inline PointCloud filter(const PointCloud& cloud, Predicate pred) {
  return detail::filterCopy(cloud, pred);
}

// Remove NaN/Inf points
[[nodiscard]] inline PointCloud removeInvalid(PointCloud&& cloud) {
  detail::filterInPlace(cloud, [&](size_t i) {
    const auto& p = cloud[i];
    return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
  });
  return std::move(cloud);
}

[[nodiscard]] inline PointCloud removeInvalid(const PointCloud& cloud) {
  return detail::filterCopy(cloud, [&](size_t i) {
    const auto& p = cloud[i];
    return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
  });
}

} // namespace filters
} // namespace nanopcl

#endif // NANOPCL_FILTERS_CORE_HPP
