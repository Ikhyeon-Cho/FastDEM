// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_FILTERS_IMPL_CROP_IMPL_HPP
#define NANOPCL_FILTERS_IMPL_CROP_IMPL_HPP

#include <cmath>

namespace nanopcl {
namespace filters {

// =============================================================================
// Box Cropping
// =============================================================================

inline PointCloud cropBox(const PointCloud& cloud,
                          const Point& min,
                          const Point& max,
                          FilterMode mode) {
  if (mode == FilterMode::INSIDE) {
    return filter(cloud, [&](size_t i) {
      const auto p = cloud.point(i);
      return p.x() >= min.x() && p.x() <= max.x() &&
             p.y() >= min.y() && p.y() <= max.y() &&
             p.z() >= min.z() && p.z() <= max.z();
    });
  }
  return filter(cloud, [&](size_t i) {
    const auto p = cloud.point(i);
    return p.x() < min.x() || p.x() > max.x() ||
           p.y() < min.y() || p.y() > max.y() ||
           p.z() < min.z() || p.z() > max.z();
  });
}

inline PointCloud cropBox(PointCloud&& cloud,
                          const Point& min,
                          const Point& max,
                          FilterMode mode) {
  if (mode == FilterMode::INSIDE) {
    return filter(std::move(cloud), [&](size_t i) {
      const auto p = cloud.point(i);
      return p.x() >= min.x() && p.x() <= max.x() &&
             p.y() >= min.y() && p.y() <= max.y() &&
             p.z() >= min.z() && p.z() <= max.z();
    });
  }
  return filter(std::move(cloud), [&](size_t i) {
    const auto p = cloud.point(i);
    return p.x() < min.x() || p.x() > max.x() ||
           p.y() < min.y() || p.y() > max.y() ||
           p.z() < min.z() || p.z() > max.z();
  });
}

// =============================================================================
// Range Cropping
// =============================================================================

inline PointCloud cropRange(const PointCloud& cloud,
                            float min_range,
                            float max_range,
                            FilterMode mode) {
  const float min_sq = min_range * min_range;
  const float max_sq = max_range * max_range;

  if (mode == FilterMode::INSIDE) {
    return filter(cloud, [&](size_t i) {
      float d2 = cloud.point(i).squaredNorm();
      return d2 >= min_sq && d2 <= max_sq;
    });
  }
  return filter(cloud, [&](size_t i) {
    float d2 = cloud.point(i).squaredNorm();
    return d2 < min_sq || d2 > max_sq;
  });
}

inline PointCloud cropRange(PointCloud&& cloud,
                            float min_range,
                            float max_range,
                            FilterMode mode) {
  const float min_sq = min_range * min_range;
  const float max_sq = max_range * max_range;

  if (mode == FilterMode::INSIDE) {
    return filter(std::move(cloud), [&](size_t i) {
      float d2 = cloud.point(i).squaredNorm();
      return d2 >= min_sq && d2 <= max_sq;
    });
  }
  return filter(std::move(cloud), [&](size_t i) {
    float d2 = cloud.point(i).squaredNorm();
    return d2 < min_sq || d2 > max_sq;
  });
}

// =============================================================================
// Single Axis Cropping
// =============================================================================

inline PointCloud cropX(const PointCloud& cloud, float min, float max, FilterMode mode) {
  if (mode == FilterMode::INSIDE) {
    return filter(cloud, [&](size_t i) {
      float v = cloud.point(i).x();
      return v >= min && v <= max;
    });
  }
  return filter(cloud, [&](size_t i) {
    float v = cloud.point(i).x();
    return v < min || v > max;
  });
}

inline PointCloud cropX(PointCloud&& cloud, float min, float max, FilterMode mode) {
  if (mode == FilterMode::INSIDE) {
    return filter(std::move(cloud), [&](size_t i) {
      float v = cloud.point(i).x();
      return v >= min && v <= max;
    });
  }
  return filter(std::move(cloud), [&](size_t i) {
    float v = cloud.point(i).x();
    return v < min || v > max;
  });
}

inline PointCloud cropY(const PointCloud& cloud, float min, float max, FilterMode mode) {
  if (mode == FilterMode::INSIDE) {
    return filter(cloud, [&](size_t i) {
      float v = cloud.point(i).y();
      return v >= min && v <= max;
    });
  }
  return filter(cloud, [&](size_t i) {
    float v = cloud.point(i).y();
    return v < min || v > max;
  });
}

inline PointCloud cropY(PointCloud&& cloud, float min, float max, FilterMode mode) {
  if (mode == FilterMode::INSIDE) {
    return filter(std::move(cloud), [&](size_t i) {
      float v = cloud.point(i).y();
      return v >= min && v <= max;
    });
  }
  return filter(std::move(cloud), [&](size_t i) {
    float v = cloud.point(i).y();
    return v < min || v > max;
  });
}

inline PointCloud cropZ(const PointCloud& cloud, float min, float max, FilterMode mode) {
  if (mode == FilterMode::INSIDE) {
    return filter(cloud, [&](size_t i) {
      float v = cloud.point(i).z();
      return v >= min && v <= max;
    });
  }
  return filter(cloud, [&](size_t i) {
    float v = cloud.point(i).z();
    return v < min || v > max;
  });
}

inline PointCloud cropZ(PointCloud&& cloud, float min, float max, FilterMode mode) {
  if (mode == FilterMode::INSIDE) {
    return filter(std::move(cloud), [&](size_t i) {
      float v = cloud.point(i).z();
      return v >= min && v <= max;
    });
  }
  return filter(std::move(cloud), [&](size_t i) {
    float v = cloud.point(i).z();
    return v < min || v > max;
  });
}

// =============================================================================
// Angle Cropping (cross-product based, ~56x faster than atan2)
// =============================================================================

inline PointCloud cropAngle(const PointCloud& cloud,
                            float min_angle,
                            float max_angle,
                            FilterMode mode) {
  const float cos_min = std::cos(min_angle), sin_min = std::sin(min_angle);
  const float cos_max = std::cos(max_angle), sin_max = std::sin(max_angle);
  const bool wrap = min_angle > max_angle;
  const float range = wrap ? (2.0f * static_cast<float>(M_PI) - (min_angle - max_angle))
                           : (max_angle - min_angle);
  const bool use_and = range < static_cast<float>(M_PI);
  const bool keep_inside = (mode == FilterMode::INSIDE);
  constexpr float eps = 1e-5f;

  return filter(cloud, [&](size_t i) {
    const auto p = cloud.point(i);
    float c_min = cos_min * p.y() - sin_min * p.x();
    float c_max = cos_max * p.y() - sin_max * p.x();
    bool in_range = use_and ? (c_min >= -eps && c_max <= eps)
                            : (c_min >= -eps || c_max <= eps);
    return in_range == keep_inside;
  });
}

inline PointCloud cropAngle(PointCloud&& cloud,
                            float min_angle,
                            float max_angle,
                            FilterMode mode) {
  const float cos_min = std::cos(min_angle), sin_min = std::sin(min_angle);
  const float cos_max = std::cos(max_angle), sin_max = std::sin(max_angle);
  const bool wrap = min_angle > max_angle;
  const float range = wrap ? (2.0f * static_cast<float>(M_PI) - (min_angle - max_angle))
                           : (max_angle - min_angle);
  const bool use_and = range < static_cast<float>(M_PI);
  const bool keep_inside = (mode == FilterMode::INSIDE);
  constexpr float eps = 1e-5f;

  return filter(std::move(cloud), [&](size_t i) {
    const auto p = cloud.point(i);
    float c_min = cos_min * p.y() - sin_min * p.x();
    float c_max = cos_max * p.y() - sin_max * p.x();
    bool in_range = use_and ? (c_min >= -eps && c_max <= eps)
                            : (c_min >= -eps || c_max <= eps);
    return in_range == keep_inside;
  });
}

} // namespace filters
} // namespace nanopcl

#endif // NANOPCL_FILTERS_IMPL_CROP_IMPL_HPP
