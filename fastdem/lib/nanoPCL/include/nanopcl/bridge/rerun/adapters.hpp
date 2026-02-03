// nanoPCL - Rerun type adapters for point cloud visualization
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_VISUALIZATION_RERUN_ADAPTERS_HPP
#define NANOPCL_VISUALIZATION_RERUN_ADAPTERS_HPP

#include "nanopcl/core/point_cloud.hpp"

#ifdef NANOPCL_USE_RERUN

#include <rerun.hpp>

#include "color_map.hpp"

namespace nanopcl::rr::adapters {

/// Build positions vector from PointCloud (shared helper)
inline std::vector<rerun::Position3D> extractPositions(const PointCloud& cloud) {
  std::vector<rerun::Position3D> positions;
  positions.reserve(cloud.size());

  const auto& points = cloud.points();
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& p = points[i];
    positions.push_back({p.x(), p.y(), p.z()});
  }

  return positions;
}

/// Convert PointCloud positions to Rerun Points3D archetype
inline rerun::Points3D toPoints3D(const PointCloud& cloud) {
  return rerun::Points3D(extractPositions(cloud));
}

/// Convert PointCloud with automatic channel detection
/// Color priority: Color > Intensity > none
inline rerun::Points3D toPoints3DWithChannels(
    const PointCloud& cloud, colormap::Type cmap = colormap::Type::TURBO) {
  auto positions = extractPositions(cloud);

  if (cloud.empty()) {
    return rerun::Points3D(std::move(positions));
  }

  // Priority: Color channel > Intensity channel
  if (cloud.hasColor()) {
    std::vector<rerun::Color> colors;
    colors.reserve(cloud.size());
    const auto& src_colors = cloud.colors();
    for (size_t i = 0; i < cloud.size(); ++i) {
      const auto& c = src_colors[i];
      colors.push_back(rerun::Color(c.r, c.g, c.b));
    }
    auto points = rerun::Points3D(std::move(positions));
    return std::move(points).with_colors(std::move(colors));
  }

  if (cloud.hasIntensity()) {
    auto colors = colormap::apply(cloud.intensities(), cmap);
    auto points = rerun::Points3D(std::move(positions));
    return std::move(points).with_colors(std::move(colors));
  }

  return rerun::Points3D(std::move(positions));
}

/// Convert PointCloud with intensity colormap
inline rerun::Points3D toPoints3DByIntensity(const PointCloud& cloud,
                                             colormap::Type cmap,
                                             float min_val = NAN,
                                             float max_val = NAN) {
  auto positions = extractPositions(cloud);

  if (cloud.empty() || !cloud.hasIntensity()) {
    return rerun::Points3D(std::move(positions));
  }

  auto colors = colormap::apply(cloud.intensities(), cmap, min_val, max_val);
  auto points = rerun::Points3D(std::move(positions));
  return std::move(points).with_colors(std::move(colors));
}

/// Convert PointCloud with Z axis colormap
inline rerun::Points3D toPoints3DByZ(const PointCloud& cloud,
                                     colormap::Type cmap = colormap::Type::TURBO,
                                     float min_z = NAN,
                                     float max_z = NAN) {
  auto positions = extractPositions(cloud);

  if (cloud.empty()) {
    return rerun::Points3D(std::move(positions));
  }

  std::vector<float> z_values;
  z_values.reserve(cloud.size());
  const auto& pts = cloud.points();
  for (size_t i = 0; i < cloud.size(); ++i) {
    z_values.push_back(pts[i].z());
  }

  auto colors = colormap::apply(z_values, cmap, min_z, max_z);
  auto points = rerun::Points3D(std::move(positions));
  return std::move(points).with_colors(std::move(colors));
}

/// Convert PointCloud with range (distance from origin) colormap
inline rerun::Points3D toPoints3DByRange(const PointCloud& cloud,
                                         colormap::Type cmap = colormap::Type::TURBO,
                                         float min_range = NAN,
                                         float max_range = NAN) {
  auto positions = extractPositions(cloud);

  if (cloud.empty()) {
    return rerun::Points3D(std::move(positions));
  }

  std::vector<float> ranges;
  ranges.reserve(cloud.size());
  const auto& pts = cloud.points();
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& p = pts[i];
    ranges.push_back(std::sqrt(p.x() * p.x() + p.y() * p.y() + p.z() * p.z()));
  }

  auto colors = colormap::apply(ranges, cmap, min_range, max_range);
  auto points = rerun::Points3D(std::move(positions));
  return std::move(points).with_colors(std::move(colors));
}

/// Convert PointCloud with ring channel colormap (for LiDAR data)
inline rerun::Points3D toPoints3DByRing(const PointCloud& cloud,
                                        colormap::Type cmap = colormap::Type::TURBO) {
  auto positions = extractPositions(cloud);

  if (cloud.empty() || !cloud.hasRing()) {
    return rerun::Points3D(std::move(positions));
  }

  auto colors = colormap::apply(cloud.rings(), cmap);
  auto points = rerun::Points3D(std::move(positions));
  return std::move(points).with_colors(std::move(colors));
}

/// Convert PointCloud with fixed color
inline rerun::Points3D toPoints3DWithColor(const PointCloud& cloud,
                                           const rerun::Color& color) {
  auto points = rerun::Points3D(extractPositions(cloud));
  return std::move(points).with_colors(color);
}

} // namespace nanopcl::rr::adapters

#endif // NANOPCL_USE_RERUN

#endif // NANOPCL_VISUALIZATION_RERUN_ADAPTERS_HPP
