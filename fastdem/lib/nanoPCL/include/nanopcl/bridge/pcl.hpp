// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Bridge module: PCL pcl::PointCloud<T> conversion.
//
// Supported PCL point types:
//   - PointXYZ, PointXYZI, PointXYZL
//   - PointXYZRGB, PointXYZRGBA, PointXYZRGBL
//   - PointXYZINormal, PointXYZRGBNormal
//   - PointNormal, PointXYZLNormal
//   - Any custom type with x, y, z members
//
// Usage:
//   #include <nanopcl/bridge/pcl.hpp>
//
//   // PCL -> nanoPCL
//   pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud = ...;
//   auto cloud = nanopcl::from(*pcl_cloud);
//
//   // nanoPCL -> PCL (using cloud's metadata)
//   auto pcl_out = nanopcl::to<pcl::PointXYZI>(cloud);
//
//   // nanoPCL -> PCL (explicit metadata)
//   auto pcl_out = nanopcl::to<pcl::PointXYZI>(cloud, "map", timestamp_us);

#ifndef NANOPCL_BRIDGE_PCL_HPP
#define NANOPCL_BRIDGE_PCL_HPP

#include <cmath>
#include <cstdint>
#include <type_traits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {

namespace detail {

// =============================================================================
// Type Traits for PCL Point Types
// =============================================================================

template <typename T, typename = void>
struct has_intensity : std::false_type {};

template <typename T>
struct has_intensity<T, std::void_t<decltype(std::declval<T>().intensity)>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_intensity_v = has_intensity<T>::value;

template <typename T, typename = void>
struct has_rgb : std::false_type {};

template <typename T>
struct has_rgb<T, std::void_t<decltype(std::declval<T>().rgb)>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_rgb_v = has_rgb<T>::value;

template <typename T, typename = void>
struct has_rgba : std::false_type {};

template <typename T>
struct has_rgba<T, std::void_t<decltype(std::declval<T>().rgba)>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_rgba_v = has_rgba<T>::value;

template <typename T, typename = void>
struct has_rgb_separate : std::false_type {};

template <typename T>
struct has_rgb_separate<
    T, std::void_t<decltype(std::declval<T>().r), decltype(std::declval<T>().g),
                   decltype(std::declval<T>().b)>> : std::true_type {};

template <typename T>
inline constexpr bool has_rgb_separate_v = has_rgb_separate<T>::value;

template <typename T, typename = void>
struct has_normal : std::false_type {};

template <typename T>
struct has_normal<T, std::void_t<decltype(std::declval<T>().normal_x),
                                 decltype(std::declval<T>().normal_y),
                                 decltype(std::declval<T>().normal_z)>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_normal_v = has_normal<T>::value;

template <typename T, typename = void>
struct has_label : std::false_type {};

template <typename T>
struct has_label<T, std::void_t<decltype(std::declval<T>().label)>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_label_v = has_label<T>::value;

template <typename T, typename = void>
struct has_curvature : std::false_type {};

template <typename T>
struct has_curvature<T, std::void_t<decltype(std::declval<T>().curvature)>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_curvature_v = has_curvature<T>::value;

template <typename T>
inline constexpr bool has_any_color_v =
    has_rgb_v<T> || has_rgba_v<T> || has_rgb_separate_v<T>;

// =============================================================================
// Color Helpers
// =============================================================================

template <typename PointT>
Color extractColor(const PointT& pt) {
  if constexpr (has_rgb_v<PointT>) {
    uint32_t rgb = *reinterpret_cast<const uint32_t*>(&pt.rgb);
    return Color{static_cast<uint8_t>((rgb >> 16) & 0xFF),
                 static_cast<uint8_t>((rgb >> 8) & 0xFF),
                 static_cast<uint8_t>(rgb & 0xFF)};
  } else if constexpr (has_rgba_v<PointT>) {
    uint32_t rgba = pt.rgba;
    return Color{static_cast<uint8_t>((rgba >> 16) & 0xFF),
                 static_cast<uint8_t>((rgba >> 8) & 0xFF),
                 static_cast<uint8_t>(rgba & 0xFF)};
  } else if constexpr (has_rgb_separate_v<PointT>) {
    return Color{pt.r, pt.g, pt.b};
  } else {
    return Color{0, 0, 0};
  }
}

template <typename PointT>
void setColor(PointT& pt, const Color& color) {
  if constexpr (has_rgb_v<PointT>) {
    uint32_t rgb = (static_cast<uint32_t>(color.r) << 16) |
                   (static_cast<uint32_t>(color.g) << 8) |
                   static_cast<uint32_t>(color.b);
    *reinterpret_cast<uint32_t*>(&pt.rgb) = rgb;
  } else if constexpr (has_rgba_v<PointT>) {
    pt.rgba = (static_cast<uint32_t>(255) << 24) |
              (static_cast<uint32_t>(color.r) << 16) |
              (static_cast<uint32_t>(color.g) << 8) |
              static_cast<uint32_t>(color.b);
  } else if constexpr (has_rgb_separate_v<PointT>) {
    pt.r = color.r;
    pt.g = color.g;
    pt.b = color.b;
  }
}

}  // namespace detail

// =============================================================================
// from() - PCL to nanoPCL
// =============================================================================

/**
 * @brief Convert PCL PointCloud to nanoPCL PointCloud
 *
 * Automatically detects and converts available fields based on PCL point type.
 *
 * @tparam PointT PCL point type (auto-deduced)
 * @param pcl_cloud PCL point cloud
 * @return PointCloud with detected channels enabled
 *
 * @note Invalid points (NaN/Inf) are filtered out.
 * @note frame_id and timestamp are stored in the returned cloud.
 */
template <typename PointT>
[[nodiscard]] PointCloud from(const pcl::PointCloud<PointT>& pcl_cloud) {
  PointCloud cloud;

  const size_t num_points = pcl_cloud.size();
  if (num_points == 0) {
    cloud.setFrameId(pcl_cloud.header.frame_id);
    cloud.setTimestamp(pcl_cloud.header.stamp * 1000ULL);  // us to ns
    return cloud;
  }

  // 1. Activate channels BEFORE adding points
  if constexpr (detail::has_intensity_v<PointT>) {
    cloud.useIntensity();
  }
  if constexpr (detail::has_any_color_v<PointT>) {
    cloud.useColor();
  }
  if constexpr (detail::has_normal_v<PointT>) {
    cloud.useNormal();
  }
  if constexpr (detail::has_label_v<PointT>) {
    cloud.useLabel();
  }

  // 2. Reserve memory
  cloud.points().reserve(num_points);
  if constexpr (detail::has_intensity_v<PointT>) {
    cloud.intensities().reserve(num_points);
  }
  if constexpr (detail::has_any_color_v<PointT>) {
    cloud.colors().reserve(num_points);
  }
  if constexpr (detail::has_normal_v<PointT>) {
    cloud.normals().reserve(num_points);
  }
  if constexpr (detail::has_label_v<PointT>) {
    cloud.labels().reserve(num_points);
  }

  // 3. Set metadata
  cloud.setFrameId(pcl_cloud.header.frame_id);
  cloud.setTimestamp(pcl_cloud.header.stamp * 1000ULL);  // us to ns

  // 4. Convert points (with NaN filtering)
  for (const auto& pt : pcl_cloud.points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }

    cloud.points().emplace_back(pt.x, pt.y, pt.z, 1.0f);

    if constexpr (detail::has_intensity_v<PointT>) {
      cloud.intensities().push_back(pt.intensity);
    }
    if constexpr (detail::has_any_color_v<PointT>) {
      cloud.colors().push_back(detail::extractColor(pt));
    }
    if constexpr (detail::has_normal_v<PointT>) {
      cloud.normals().emplace_back(pt.normal_x, pt.normal_y, pt.normal_z, 0.0f);
    }
    if constexpr (detail::has_label_v<PointT>) {
      cloud.labels().push_back(Label(pt.label));
    }
  }

  return cloud;
}

// =============================================================================
// to() - nanoPCL to PCL
// =============================================================================

/**
 * @brief Convert nanoPCL PointCloud to PCL PointCloud
 *
 * Uses the cloud's internal frame_id and timestamp.
 *
 * @tparam PointT Target PCL point type (must be specified)
 * @param cloud nanoPCL point cloud
 * @return PCL PointCloud
 *
 * @note Only channels supported by PointT are written.
 */
template <typename PointT>
[[nodiscard]] pcl::PointCloud<PointT> to(const PointCloud& cloud) {
  return to<PointT>(cloud, cloud.frameId(), cloud.timestamp() / 1000ULL);
}

/**
 * @brief Convert nanoPCL PointCloud to PCL PointCloud with explicit metadata
 *
 * @tparam PointT Target PCL point type (must be specified)
 * @param cloud nanoPCL point cloud
 * @param frame_id Frame ID for PCL header
 * @param timestamp_us Timestamp in microseconds for PCL header
 * @return PCL PointCloud
 *
 * @note Only channels supported by PointT are written.
 */
template <typename PointT>
[[nodiscard]] pcl::PointCloud<PointT> to(const PointCloud& cloud,
                                          const std::string& frame_id,
                                          uint64_t timestamp_us) {
  pcl::PointCloud<PointT> pcl_cloud;

  // Set metadata
  pcl_cloud.header.frame_id = frame_id;
  pcl_cloud.header.stamp = timestamp_us;

  // Set dimensions
  pcl_cloud.width = static_cast<uint32_t>(cloud.size());
  pcl_cloud.height = 1;
  pcl_cloud.is_dense = true;

  if (cloud.empty()) {
    return pcl_cloud;
  }

  pcl_cloud.points.resize(cloud.size());

  // Cache channel availability (avoid repeated checks in loop)
  [[maybe_unused]] const bool has_intensity = cloud.hasIntensity();
  [[maybe_unused]] const bool has_color = cloud.hasColor();
  [[maybe_unused]] const bool has_normal = cloud.hasNormal();
  [[maybe_unused]] const bool has_label = cloud.hasLabel();

  // Convert points
  for (size_t i = 0; i < cloud.size(); ++i) {
    auto& pt = pcl_cloud.points[i];
    const auto& p = cloud[i];

    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();

    if constexpr (detail::has_intensity_v<PointT>) {
      pt.intensity = has_intensity ? cloud.intensity(i) : 0.0f;
    }

    if constexpr (detail::has_any_color_v<PointT>) {
      detail::setColor(pt, has_color ? cloud.color(i) : Color{255, 255, 255});
    }

    if constexpr (detail::has_normal_v<PointT>) {
      if (has_normal) {
        const auto& n = cloud.normals()[i];
        pt.normal_x = n.x();
        pt.normal_y = n.y();
        pt.normal_z = n.z();
      } else {
        pt.normal_x = pt.normal_y = pt.normal_z = 0.0f;
      }

      if constexpr (detail::has_curvature_v<PointT>) {
        pt.curvature = 0.0f;
      }
    }

    if constexpr (detail::has_label_v<PointT>) {
      pt.label = has_label ? cloud.label(i).val : 0;
    }
  }

  return pcl_cloud;
}

}  // namespace nanopcl

#endif  // NANOPCL_BRIDGE_PCL_HPP
