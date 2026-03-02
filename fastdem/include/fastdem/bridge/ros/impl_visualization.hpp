// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_BRIDGE_ROS_IMPL_VISUALIZATION_HPP
#define FASTDEM_BRIDGE_ROS_IMPL_VISUALIZATION_HPP

#include <fastdem/elevation_map.hpp>
#include <fastdem/postprocess/feature_extraction.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace fastdem::detail {

// ── toNormalMarkersImpl ─────────────────────────────────────────────────────
//
// Converts ElevationMap normal vectors to a MarkerArray containing:
//   [0] DELETEALL marker  — clears stale markers from previous frame
//   [1] LINE_LIST marker  — one line per cell (start → start + normal * length)
//
// Requires layers: elevation, _normal_x, _normal_y, _normal_z, slope.
// Color: slope-based heatmap (green=flat → red=steep).
//
template <typename MarkerArrayT, typename MarkerT, typename PointT,
          typename ColorT, typename TimeT>
MarkerArrayT toNormalMarkersImpl(const ElevationMap& map, const TimeT& stamp,
                                 float arrow_length = 0.15f, int stride = 1) {
  MarkerArrayT marker_array;

  // Validate required layers
  if (!map.exists(layer::elevation) || !map.exists(layer::normal_x) ||
      !map.exists(layer::normal_y) || !map.exists(layer::normal_z)) {
    return marker_array;
  }

  const bool has_slope = map.exists(layer::slope);

  // [0] DELETEALL marker to clear previous frame
  {
    MarkerT del;
    del.header.stamp = stamp;
    del.header.frame_id = map.getFrameId();
    del.action = MarkerT::DELETEALL;
    marker_array.markers.push_back(del);
  }

  // Prepare data access
  const auto& elev_mat = map.get(layer::elevation);
  const auto& nx_mat = map.get(layer::normal_x);
  const auto& ny_mat = map.get(layer::normal_y);
  const auto& nz_mat = map.get(layer::normal_z);
  const float* slope_data =
      has_slope ? map.get(layer::slope).data() : nullptr;

  const double res = map.getResolution();
  const auto size = map.getSize();
  const Eigen::Index rows = size(0);
  const Eigen::Index cols = size(1);

  const double origin_x =
      map.getPosition().x() + map.getLength().x() / 2.0 - res / 2.0;
  const double origin_y =
      map.getPosition().y() + map.getLength().y() / 2.0 - res / 2.0;

  // [1] LINE_LIST marker with all normal vectors
  MarkerT lines;
  lines.header.stamp = stamp;
  lines.header.frame_id = map.getFrameId();
  lines.ns = "normals";
  lines.id = 0;
  lines.type = MarkerT::LINE_LIST;
  lines.action = MarkerT::ADD;
  lines.scale.x = 0.01;  // line width
  lines.pose.orientation.w = 1.0;

  // Ensure stride >= 1
  stride = std::max(stride, 1);

  // Reserve approximate capacity
  const int est_count =
      (rows / stride + 1) * (cols / stride + 1);
  lines.points.reserve(est_count * 2);
  lines.colors.reserve(est_count * 2);

  for (auto cell : map.cells()) {
    if (cell.row % stride != 0 || cell.col % stride != 0) continue;

    const float z = elev_mat(cell.index);
    const float nx = nx_mat(cell.index);
    const float ny = ny_mat(cell.index);
    const float nz = nz_mat(cell.index);

    if (!std::isfinite(z) || !std::isfinite(nx)) continue;

    // World coordinates from logical cell coords
    const float x = static_cast<float>(origin_x - cell.row * res);
    const float y = static_cast<float>(origin_y - cell.col * res);

    // Start point (surface)
    PointT p0;
    p0.x = x;
    p0.y = y;
    p0.z = z;

    // End point (surface + normal * length)
    PointT p1;
    p1.x = x + nx * arrow_length;
    p1.y = y + ny * arrow_length;
    p1.z = z + nz * arrow_length;

    lines.points.push_back(p0);
    lines.points.push_back(p1);

    // Color by slope: green (flat) → red (steep)
    float t = 0.0f;
    if (slope_data) {
      const float slope_deg = slope_data[cell.index];
      if (std::isfinite(slope_deg)) {
        t = std::clamp(slope_deg / 45.0f, 0.0f, 1.0f);
      }
    }

    ColorT color;
    color.r = t;
    color.g = 0.8f * (1.0f - t);
    color.b = 0.0f;
    color.a = 0.8f;

    // Same color for both vertices of the line
    lines.colors.push_back(color);
    lines.colors.push_back(color);
  }

  marker_array.markers.push_back(lines);
  return marker_array;
}

}  // namespace fastdem::detail

#endif  // FASTDEM_BRIDGE_ROS_IMPL_VISUALIZATION_HPP
