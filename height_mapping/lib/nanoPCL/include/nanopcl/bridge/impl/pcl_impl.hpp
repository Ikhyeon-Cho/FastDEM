// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Implementation details for PCL bridge.
// Do not include this file directly; include <nanopcl/bridge/pcl.hpp>

#ifndef NANOPCL_BRIDGE_IMPL_PCL_IMPL_HPP
#define NANOPCL_BRIDGE_IMPL_PCL_IMPL_HPP

#include <cmath>
#include <type_traits>

namespace npcl {

namespace detail {

// =============================================================================
// Type Traits for PCL Point Types
// =============================================================================

// Check for intensity member
template <typename T, typename = void>
struct has_intensity : std::false_type {};

template <typename T>
struct has_intensity<T, std::void_t<decltype(std::declval<T>().intensity)>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_intensity_v = has_intensity<T>::value;

// Check for rgb member (packed uint32_t)
template <typename T, typename = void>
struct has_rgb : std::false_type {};

template <typename T>
struct has_rgb<T, std::void_t<decltype(std::declval<T>().rgb)>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_rgb_v = has_rgb<T>::value;

// Check for rgba member
template <typename T, typename = void>
struct has_rgba : std::false_type {};

template <typename T>
struct has_rgba<T, std::void_t<decltype(std::declval<T>().rgba)>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_rgba_v = has_rgba<T>::value;

// Check for r, g, b members (some PCL types have separate members)
template <typename T, typename = void>
struct has_rgb_separate : std::false_type {};

template <typename T>
struct has_rgb_separate<
    T, std::void_t<decltype(std::declval<T>().r), decltype(std::declval<T>().g),
                   decltype(std::declval<T>().b)>> : std::true_type {};

template <typename T>
inline constexpr bool has_rgb_separate_v = has_rgb_separate<T>::value;

// Check for normal members
template <typename T, typename = void>
struct has_normal : std::false_type {};

template <typename T>
struct has_normal<T, std::void_t<decltype(std::declval<T>().normal_x),
                                 decltype(std::declval<T>().normal_y),
                                 decltype(std::declval<T>().normal_z)>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_normal_v = has_normal<T>::value;

// Check for label member
template <typename T, typename = void>
struct has_label : std::false_type {};

template <typename T>
struct has_label<T, std::void_t<decltype(std::declval<T>().label)>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_label_v = has_label<T>::value;

// Check for curvature member
template <typename T, typename = void>
struct has_curvature : std::false_type {};

template <typename T>
struct has_curvature<T, std::void_t<decltype(std::declval<T>().curvature)>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_curvature_v = has_curvature<T>::value;

// =============================================================================
// Color Extraction Helpers
// =============================================================================

template <typename PointT>
Color extractColor(const PointT& pt) {
  if constexpr (has_rgb_v<PointT>) {
    // PCL packs RGB as uint32_t: 0x00RRGGBB
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
    uint32_t rgb = (static_cast<uint32_t>(color[0]) << 16) |
                   (static_cast<uint32_t>(color[1]) << 8) |
                   static_cast<uint32_t>(color[2]);
    *reinterpret_cast<uint32_t*>(&pt.rgb) = rgb;
  } else if constexpr (has_rgba_v<PointT>) {
    pt.rgba = (static_cast<uint32_t>(255) << 24) |  // Alpha = 255
              (static_cast<uint32_t>(color[0]) << 16) |
              (static_cast<uint32_t>(color[1]) << 8) |
              static_cast<uint32_t>(color[2]);
  } else if constexpr (has_rgb_separate_v<PointT>) {
    pt.r = color[0];
    pt.g = color[1];
    pt.b = color[2];
  }
}

// Check if point type has any color field
template <typename T>
inline constexpr bool has_any_color_v =
    has_rgb_v<T> || has_rgba_v<T> || has_rgb_separate_v<T>;

}  // namespace detail

// =============================================================================
// Convert Specialization for pcl::PointCloud<PointT>
// =============================================================================

template <typename PointT>
struct Convert<pcl::PointCloud<PointT>> {
  using PclCloud = pcl::PointCloud<PointT>;

  static PointCloud from(const PclCloud& pcl_cloud) {
    const size_t num_points = pcl_cloud.size();
    PointCloud cloud(num_points);

    // Set metadata
    cloud.setFrameId(pcl_cloud.header.frame_id);
    cloud.setTimestamp(pcl_cloud.header.stamp * 1000ULL);  // us to ns

    if (num_points == 0) {
      return cloud;
    }

    // Enable channels based on point type
    if constexpr (detail::has_intensity_v<PointT>) {
      cloud.enableIntensity();
      cloud.intensity().reserve(num_points);
    }
    if constexpr (detail::has_any_color_v<PointT>) {
      cloud.enableColor();
      cloud.color().reserve(num_points);
    }
    if constexpr (detail::has_normal_v<PointT>) {
      cloud.enableNormal();
      cloud.normal().reserve(num_points);
    }
    if constexpr (detail::has_label_v<PointT>) {
      cloud.enableLabel();
      cloud.label().reserve(num_points);
    }

    cloud.xyz().reserve(num_points);

    // Convert points
    for (const auto& pt : pcl_cloud.points) {
      // Skip invalid points
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
          !std::isfinite(pt.z)) {
        continue;
      }

      cloud.xyz().emplace_back(pt.x, pt.y, pt.z);

      if constexpr (detail::has_intensity_v<PointT>) {
        cloud.intensity().push_back(pt.intensity);
      }
      if constexpr (detail::has_any_color_v<PointT>) {
        cloud.color().push_back(detail::extractColor(pt));
      }
      if constexpr (detail::has_normal_v<PointT>) {
        cloud.normal().emplace_back(pt.normal_x, pt.normal_y, pt.normal_z);
      }
      if constexpr (detail::has_label_v<PointT>) {
        cloud.label().push_back(pt.label);
      }
    }

    return cloud;
  }

  static PclCloud to(const PointCloud& cloud) {
    PclCloud pcl_cloud;

    // Set metadata
    pcl_cloud.header.frame_id = cloud.frameId();
    pcl_cloud.header.stamp = cloud.timestamp() / 1000ULL;  // ns to us

    // Set dimensions
    pcl_cloud.width = static_cast<uint32_t>(cloud.size());
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = true;

    if (cloud.empty()) {
      return pcl_cloud;
    }

    pcl_cloud.points.resize(cloud.size());

    // Convert points
    for (size_t i = 0; i < cloud.size(); ++i) {
      auto& pt = pcl_cloud.points[i];
      const auto& p = cloud[i];

      pt.x = p.x();
      pt.y = p.y();
      pt.z = p.z();

      if constexpr (detail::has_intensity_v<PointT>) {
        if (cloud.hasIntensity()) {
          pt.intensity = cloud.intensity()[i];
        } else {
          pt.intensity = 0.0f;
        }
      }

      if constexpr (detail::has_any_color_v<PointT>) {
        if (cloud.hasColor()) {
          detail::setColor(pt, cloud.color()[i]);
        } else {
          detail::setColor(pt, Color{255, 255, 255});
        }
      }

      if constexpr (detail::has_normal_v<PointT>) {
        if (cloud.hasNormal()) {
          const auto& n = cloud.normal()[i];
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
        if (cloud.hasLabel()) {
          pt.label = cloud.label()[i];
        } else {
          pt.label = 0;
        }
      }
    }

    return pcl_cloud;
  }
};

// =============================================================================
// Convenience Overloads for PCL Smart Pointers
// =============================================================================

template <typename PointT>
[[nodiscard]] inline PointCloud from(
    const typename pcl::PointCloud<PointT>::Ptr& pcl_cloud) {
  return Convert<pcl::PointCloud<PointT>>::from(*pcl_cloud);
}

template <typename PointT>
[[nodiscard]] inline PointCloud from(
    const typename pcl::PointCloud<PointT>::ConstPtr& pcl_cloud) {
  return Convert<pcl::PointCloud<PointT>>::from(*pcl_cloud);
}

template <typename PointT>
[[nodiscard]] inline typename pcl::PointCloud<PointT>::Ptr toPclPtr(
    const PointCloud& cloud) {
  typename pcl::PointCloud<PointT>::Ptr pcl_cloud(
      new pcl::PointCloud<PointT>());
  *pcl_cloud = Convert<pcl::PointCloud<PointT>>::to(cloud);
  return pcl_cloud;
}

}  // namespace npcl

#endif  // NANOPCL_BRIDGE_IMPL_PCL_IMPL_HPP
