// nanoPCL - Rerun visualization module
// SPDX-License-Identifier: MIT
//
// Usage:
//   #include <nanopcl/bridge/rerun.hpp>
//
//   auto cloud = nanopcl::io::loadKITTI("scan.bin");
//   nanopcl::rr::showCloud("world/scan", cloud);  // One-liner visualization

#ifndef NANOPCL_BRIDGE_RERUN_HPP
#define NANOPCL_BRIDGE_RERUN_HPP

#include "nanopcl/core/point_cloud.hpp"

#ifdef NANOPCL_USE_RERUN

#include <utility>

#include "rerun/adapters.hpp"
#include "rerun/color_map.hpp"
#include "rerun/helpers.hpp"
#include "rerun/stream.hpp"

namespace nanopcl::rr {

/// Point visualization style options
struct PointStyle {
  float radius = 0.0f; ///< Point radius (0 = Rerun default)
  std::optional<rerun::Color> color; ///< Fixed color (nullopt = auto from channels)
  colormap::Type colormap = colormap::Type::TURBO; ///< Colormap for intensity
};

/// Coloring mode for point cloud visualization
enum class ColorMode {
  Auto,        ///< Use Color > Intensity > Z priority
  ByIntensity, ///< Color by intensity channel
  ByZ,         ///< Color by Z axis value
  ByRing,      ///< Color by ring channel (LiDAR)
  ByRange,     ///< Color by distance from origin
  Fixed        ///< Use fixed color from PointStyle
};

// ============================================================================
// Internal: log entity with optional radius
// ============================================================================
namespace detail {

inline void logEntity(const std::string& path, rerun::Points3D&& entity,
                      float radius) {
  if (radius > 0) {
    stream().log(path, std::move(entity).with_radii({radius}));
  } else {
    stream().log(path, std::move(entity));
  }
}

} // namespace detail

// ============================================================================
// Primary API: One-liner visualization
// ============================================================================

/// Visualize a point cloud with automatic channel detection
inline void showCloud(const std::string& path, const PointCloud& cloud,
                      const PointStyle& style = {}) {
  if (cloud.empty()) return;

  if (style.color.has_value()) {
    detail::logEntity(path, adapters::toPoints3DWithColor(cloud, *style.color),
                      style.radius);
  } else {
    detail::logEntity(path,
                      adapters::toPoints3DWithChannels(cloud, style.colormap),
                      style.radius);
  }
}

/// Visualize with fixed color (convenience overload)
inline void showCloud(const std::string& path, const PointCloud& cloud,
                      const rerun::Color& color) {
  showCloud(path, cloud, PointStyle{.color = color});
}

/// Visualize with explicit coloring mode
inline void showCloud(const std::string& path, const PointCloud& cloud,
                      ColorMode mode, const PointStyle& style = {}) {
  if (cloud.empty()) return;

  auto buildEntity = [&]() -> rerun::Points3D {
    switch (mode) {
    case ColorMode::ByIntensity:
      return adapters::toPoints3DByIntensity(cloud, style.colormap);
    case ColorMode::ByZ:
      return adapters::toPoints3DByZ(cloud, style.colormap);
    case ColorMode::ByRing:
      return adapters::toPoints3DByRing(cloud, style.colormap);
    case ColorMode::ByRange:
      return adapters::toPoints3DByRange(cloud, style.colormap);
    case ColorMode::Fixed:
      if (style.color.has_value()) {
        return adapters::toPoints3DWithColor(cloud, *style.color);
      }
      return adapters::toPoints3D(cloud);
    case ColorMode::Auto:
    default:
      return adapters::toPoints3DWithChannels(cloud, style.colormap);
    }
  };

  detail::logEntity(path, buildEntity(), style.radius);
}

// ============================================================================
// Timeline support for sequences (SLAM, temporal data)
// ============================================================================

/// Visualize with explicit timestamp (for temporal sequences)
inline void showCloud(const std::string& path, const PointCloud& cloud,
                      double timestamp_sec, const PointStyle& style = {}) {
  stream().set_time_timestamp_secs_since_epoch("time", timestamp_sec);
  showCloud(path, cloud, style);
}

/// Visualize with sequence number (for frame-based sequences)
inline void showCloud(const std::string& path, const PointCloud& cloud,
                      int64_t sequence, const PointStyle& style = {}) {
  stream().set_time_sequence("frame", sequence);
  showCloud(path, cloud, style);
}

// ============================================================================
// Direct Rerun entity logging (for advanced users)
// ============================================================================

/// Log any Rerun archetype directly
template <typename T>
inline void log(const std::string& path, T&& archetype) {
  stream().log(path, std::forward<T>(archetype));
}

/// Log text annotation
inline void logText(const std::string& path, const std::string& text) {
  stream().log(path, rerun::TextLog(text));
}

} // namespace nanopcl::rr

#else // !NANOPCL_USE_RERUN

// ============================================================================
// Stub implementation when Rerun is disabled
// ============================================================================

#include <vector>

namespace nanopcl::rr {

enum class ConnectMode { SPAWN, CONNECT, SAVE };
enum class ColorMode { Auto, ByIntensity, ByZ, ByRing, ByRange, Fixed };

namespace colormap {
enum class Type { TURBO, VIRIDIS, GRAYSCALE, JET };
} // namespace colormap

struct PointStyle {
  float radius = 0.0f;
  colormap::Type colormap = colormap::Type::TURBO;
};

inline void init(const std::string& = "", ConnectMode = ConnectMode::SPAWN,
                 const std::string& = "") {}
inline void showCloud(const std::string&, const PointCloud&, const PointStyle& = {}) {}
inline void showCloud(const std::string&, const PointCloud&, ColorMode,
                      const PointStyle& = {}) {}
inline void showCloud(const std::string&, const PointCloud&, double,
                      const PointStyle& = {}) {}
inline void showCloud(const std::string&, const PointCloud&, int64_t,
                      const PointStyle& = {}) {}
inline void logText(const std::string&, const std::string&) {}

} // namespace nanopcl::rr

#endif // NANOPCL_USE_RERUN

#endif // NANOPCL_BRIDGE_RERUN_HPP
