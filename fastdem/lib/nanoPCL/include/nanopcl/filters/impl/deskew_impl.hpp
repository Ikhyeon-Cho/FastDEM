// nanoPCL - Deskew implementation
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_IMPL_DESKEW_IMPL_HPP
#define NANOPCL_FILTERS_IMPL_DESKEW_IMPL_HPP

#include <algorithm>
#include <stdexcept>

#include "nanopcl/core/math.hpp"

namespace nanopcl {
namespace filters {

namespace detail {

/// @brief Core deskew implementation
/// @param cloud Point cloud to modify in-place
/// @param pose_lookup Function returning pose at given timestamp
/// @param T_end Reference pose (all points transformed to this frame)
inline void deskewInPlace(PointCloud& cloud,
                          const PoseLookupFunc& pose_lookup,
                          const Eigen::Isometry3d& T_end) {
  if (cloud.empty())
    return;

  const Eigen::Isometry3d T_end_inv = T_end.inverse();
  const Eigen::Matrix3d R_end_inv = T_end_inv.rotation();
  auto& points = cloud.points();
  const size_t n = points.size();

  // Transform points
  for (size_t i = 0; i < n; ++i) {
    const float t = cloud.hasTime() ? cloud.time(i) : static_cast<float>(i) / (n - 1);
    const Eigen::Isometry3d T_point = pose_lookup(static_cast<double>(t));

    // Relative transform from T_point to T_end
    Eigen::Isometry3d T_rel;
    T_rel.matrix() = T_end_inv.matrix() * T_point.matrix();

    // Transform point
    Eigen::Vector3d p(points[i].x(), points[i].y(), points[i].z());
    p = T_rel * p;
    points[i] = Point4(static_cast<float>(p.x()), static_cast<float>(p.y()),
                       static_cast<float>(p.z()), points[i].w());
  }

  // Transform normals (rotation only)
  if (cloud.hasNormal()) {
    auto& normals = cloud.normals();
    for (size_t i = 0; i < n; ++i) {
      const float t = cloud.hasTime() ? cloud.time(i) : static_cast<float>(i) / (n - 1);
      const Eigen::Isometry3d T_point = pose_lookup(static_cast<double>(t));
      const Eigen::Matrix3d R_rel = R_end_inv * T_point.rotation();

      Eigen::Vector3d normal(normals[i].x(), normals[i].y(), normals[i].z());
      normal = R_rel * normal;
      normals[i] = Normal4(static_cast<float>(normal.x()),
                           static_cast<float>(normal.y()),
                           static_cast<float>(normal.z()), 0.0f);
    }
  }
}

/// @brief Get time range from cloud
/// @return pair of (min_time, max_time)
inline std::pair<double, double> getTimeRange(const PointCloud& cloud) {
  if (!cloud.hasTime() || cloud.empty()) {
    throw std::runtime_error(
        "deskew: CHANNEL strategy requires time channel. "
        "Use TimeStrategy::INDEX or enable time channel.");
  }

  const auto& times = cloud.times();
  auto [min_it, max_it] = std::minmax_element(times.begin(), times.end());
  return {static_cast<double>(*min_it), static_cast<double>(*max_it)};
}

/// @brief Create pose lookup function for linear interpolation
inline PoseLookupFunc makeLinearLookup(const Eigen::Isometry3d& T_start,
                                       const Eigen::Isometry3d& T_end,
                                       double t_start,
                                       double t_end) {
  const double dt = t_end - t_start;
  if (std::abs(dt) < 1e-9) {
    // No motion - return identity relative transform
    return [T_end](double) { return T_end; };
  }

  return [=](double t) -> Eigen::Isometry3d {
    double alpha = (t - t_start) / dt;
    alpha = std::clamp(alpha, 0.0, 1.0);
    return math::slerp(T_start, T_end, alpha);
  };
}

} // namespace detail

// =============================================================================
// Callback-based API
// =============================================================================

inline PointCloud deskew(const PointCloud& cloud, PoseLookupFunc pose_lookup) {
  PointCloud result = cloud;

  if (result.empty())
    return result;

  // Get T_end from the last point's timestamp
  double t_last;
  if (result.hasTime()) {
    const auto& times = result.times();
    t_last = static_cast<double>(*std::max_element(times.begin(), times.end()));
  } else {
    t_last = 1.0; // Assume normalized
  }

  Eigen::Isometry3d T_end = pose_lookup(t_last);
  detail::deskewInPlace(result, pose_lookup, T_end);
  return result;
}

inline PointCloud deskew(PointCloud&& cloud, PoseLookupFunc pose_lookup) {
  if (cloud.empty())
    return std::move(cloud);

  double t_last;
  if (cloud.hasTime()) {
    const auto& times = cloud.times();
    t_last = static_cast<double>(*std::max_element(times.begin(), times.end()));
  } else {
    t_last = 1.0;
  }

  Eigen::Isometry3d T_end = pose_lookup(t_last);
  detail::deskewInPlace(cloud, pose_lookup, T_end);
  return std::move(cloud);
}

// =============================================================================
// Linear Interpolation API (Auto-range)
// =============================================================================

inline PointCloud deskew(const PointCloud& cloud,
                         const Eigen::Isometry3d& T_start,
                         const Eigen::Isometry3d& T_end,
                         TimeStrategy strategy) {
  PointCloud result = cloud;

  if (result.empty())
    return result;

  double t_start_val, t_end_val;

  if (strategy == TimeStrategy::INDEX) {
    t_start_val = 0.0;
    t_end_val = 1.0;
    // For INDEX strategy, we generate time values internally
    // The lookup function will receive index-based ratios
  } else {
    // CHANNEL strategy: auto-detect range
    auto [t_min, t_max] = detail::getTimeRange(result);
    t_start_val = t_min;
    t_end_val = t_max;
  }

  auto pose_lookup =
      detail::makeLinearLookup(T_start, T_end, t_start_val, t_end_val);

  // For INDEX strategy, temporarily override time interpretation
  if (strategy == TimeStrategy::INDEX) {
    const size_t n = result.size();
    auto index_lookup = [&](double t) -> Eigen::Isometry3d {
      // t here is either from time channel or will be computed as index ratio
      return pose_lookup(t);
    };

    // Use index-based time
    const Eigen::Isometry3d T_end_ref = T_end;
    const Eigen::Isometry3d T_end_inv = T_end_ref.inverse();
    auto& points = result.points();

    for (size_t i = 0; i < n; ++i) {
      double alpha = static_cast<double>(i) / (n - 1);
      Eigen::Isometry3d T_point = math::slerp(T_start, T_end, alpha);

      Eigen::Isometry3d T_rel;
      T_rel.matrix() = T_end_inv.matrix() * T_point.matrix();

      Eigen::Vector3d p(points[i].x(), points[i].y(), points[i].z());
      p = T_rel * p;
      points[i] = Point4(static_cast<float>(p.x()), static_cast<float>(p.y()),
                         static_cast<float>(p.z()), points[i].w());
    }

    // Handle normals for INDEX strategy
    if (result.hasNormal()) {
      const Eigen::Matrix3d R_end_inv = T_end_inv.rotation();
      auto& normals = result.normals();
      for (size_t i = 0; i < n; ++i) {
        double alpha = static_cast<double>(i) / (n - 1);
        Eigen::Isometry3d T_point = math::slerp(T_start, T_end, alpha);
        Eigen::Matrix3d R_rel = R_end_inv * T_point.rotation();

        Eigen::Vector3d normal(normals[i].x(), normals[i].y(), normals[i].z());
        normal = R_rel * normal;
        normals[i] = Point4(static_cast<float>(normal.x()),
                            static_cast<float>(normal.y()),
                            static_cast<float>(normal.z()), 0.0f);
      }
    }

    return result;
  }

  // CHANNEL strategy
  detail::deskewInPlace(result, pose_lookup, T_end);
  return result;
}

inline PointCloud deskew(PointCloud&& cloud,
                         const Eigen::Isometry3d& T_start,
                         const Eigen::Isometry3d& T_end,
                         TimeStrategy strategy) {
  if (cloud.empty())
    return std::move(cloud);

  // Delegate to copy version logic but with move semantics
  double t_start_val, t_end_val;

  if (strategy == TimeStrategy::INDEX) {
    t_start_val = 0.0;
    t_end_val = 1.0;
  } else {
    auto [t_min, t_max] = detail::getTimeRange(cloud);
    t_start_val = t_min;
    t_end_val = t_max;
  }

  auto pose_lookup =
      detail::makeLinearLookup(T_start, T_end, t_start_val, t_end_val);

  if (strategy == TimeStrategy::INDEX) {
    const size_t n = cloud.size();
    const Eigen::Isometry3d T_end_inv = T_end.inverse();
    auto& points = cloud.points();

    for (size_t i = 0; i < n; ++i) {
      double alpha = static_cast<double>(i) / (n - 1);
      Eigen::Isometry3d T_point = math::slerp(T_start, T_end, alpha);

      Eigen::Isometry3d T_rel;
      T_rel.matrix() = T_end_inv.matrix() * T_point.matrix();

      Eigen::Vector3d p(points[i].x(), points[i].y(), points[i].z());
      p = T_rel * p;
      points[i] = Point4(static_cast<float>(p.x()), static_cast<float>(p.y()),
                         static_cast<float>(p.z()), points[i].w());
    }

    if (cloud.hasNormal()) {
      const Eigen::Matrix3d R_end_inv = T_end_inv.rotation();
      auto& normals = cloud.normals();
      for (size_t i = 0; i < n; ++i) {
        double alpha = static_cast<double>(i) / (n - 1);
        Eigen::Isometry3d T_point = math::slerp(T_start, T_end, alpha);
        Eigen::Matrix3d R_rel = R_end_inv * T_point.rotation();

        Eigen::Vector3d normal(normals[i].x(), normals[i].y(), normals[i].z());
        normal = R_rel * normal;
        normals[i] = Point4(static_cast<float>(normal.x()),
                            static_cast<float>(normal.y()),
                            static_cast<float>(normal.z()), 0.0f);
      }
    }

    return std::move(cloud);
  }

  detail::deskewInPlace(cloud, pose_lookup, T_end);
  return std::move(cloud);
}

// =============================================================================
// Linear Interpolation API (Explicit range)
// =============================================================================

inline PointCloud deskew(const PointCloud& cloud,
                         const Eigen::Isometry3d& T_start,
                         const Eigen::Isometry3d& T_end,
                         double t_start,
                         double t_end) {
  PointCloud result = cloud;

  if (result.empty())
    return result;

  if (!result.hasTime()) {
    throw std::runtime_error(
        "deskew: Explicit time range requires time channel.");
  }

  auto pose_lookup = detail::makeLinearLookup(T_start, T_end, t_start, t_end);
  detail::deskewInPlace(result, pose_lookup, T_end);
  return result;
}

inline PointCloud deskew(PointCloud&& cloud,
                         const Eigen::Isometry3d& T_start,
                         const Eigen::Isometry3d& T_end,
                         double t_start,
                         double t_end) {
  if (cloud.empty())
    return std::move(cloud);

  if (!cloud.hasTime()) {
    throw std::runtime_error(
        "deskew: Explicit time range requires time channel.");
  }

  auto pose_lookup = detail::makeLinearLookup(T_start, T_end, t_start, t_end);
  detail::deskewInPlace(cloud, pose_lookup, T_end);
  return std::move(cloud);
}

} // namespace filters
} // namespace nanopcl

#endif // NANOPCL_FILTERS_IMPL_DESKEW_IMPL_HPP
