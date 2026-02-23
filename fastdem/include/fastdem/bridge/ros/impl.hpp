// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_BRIDGE_ROS_IMPL_HPP
#define FASTDEM_BRIDGE_ROS_IMPL_HPP

#include <fastdem/elevation_map.hpp>

#include <grid_map_core/SubmapGeometry.hpp>

#include <cstring>
#include <string>
#include <vector>

namespace fastdem::detail {

// ── toPointCloud2Impl ────────────────────────────────────────────────────────
//
// Converts ElevationMap to PointCloud2 with ALL non-internal layers as fields.
//   elevation → z coordinate
//   color     → rgb (PCL packed float convention)
//   others    → named FLOAT32 fields
//
// Accepts optional submap region (sub_start, sub_size) to serialize only a
// portion of the map without intermediate GridMap allocation.  When sub_size
// equals the full buffer size, the function degenerates to the full-map path.
//
template <typename PointCloud2T, typename PointFieldT, typename TimeT>
PointCloud2T toPointCloud2Impl(const ElevationMap& map, const TimeT& stamp,
                               const char* elevation_layer,
                               const grid_map::Index& sub_start,
                               const grid_map::Size& sub_size) {
  const auto& elev = map.get(elevation_layer);
  const auto size = map.getSize();
  const Eigen::Index rows = size(0);
  const Eigen::Index cols = size(1);
  const Eigen::Index sub_rows = sub_size(0);
  const Eigen::Index sub_cols = sub_size(1);
  const auto startIdx = map.getStartIndex();
  const double res = map.getResolution();

  // Precompute world coordinates for each buffer row/col in the submap region
  const double origin_x =
      map.getPosition().x() + map.getLength().x() / 2.0 - res / 2.0;
  const double origin_y =
      map.getPosition().y() + map.getLength().y() / 2.0 - res / 2.0;

  std::vector<float> row_x(sub_rows);
  std::vector<Eigen::Index> buf_row(sub_rows);
  for (Eigen::Index i = 0; i < sub_rows; ++i) {
    Eigen::Index r = (sub_start(0) + i) % rows;
    buf_row[i] = r;
    int unwrapped = (r - startIdx(0) + rows) % rows;
    row_x[i] = static_cast<float>(origin_x - unwrapped * res);
  }

  std::vector<float> col_y(sub_cols);
  std::vector<Eigen::Index> buf_col(sub_cols);
  for (Eigen::Index j = 0; j < sub_cols; ++j) {
    Eigen::Index c = (sub_start(1) + j) % cols;
    buf_col[j] = c;
    int unwrapped = (c - startIdx(1) + cols) % cols;
    col_y[j] = static_cast<float>(origin_y - unwrapped * res);
  }

  // Collect layers: elevation → z, color → rgb (packed), rest → float fields
  std::vector<std::string> float_layers;
  bool has_color = false;
  for (const auto& l : map.getLayers()) {
    if (layer::isInternal(l)) continue;
    if (l == elevation_layer) continue;
    if (l == layer::color) {
      has_color = true;
      continue;
    }
    float_layers.push_back(l);
  }

  // Build PointCloud2 field descriptors (all fields are 4 bytes)
  PointCloud2T msg;
  uint32_t offset = 0;
  auto addField = [&](const std::string& name, uint8_t datatype) {
    PointFieldT f;
    f.name = name;
    f.offset = offset;
    f.datatype = datatype;
    f.count = 1;
    msg.fields.push_back(f);
    offset += 4;
  };

  addField("x", PointFieldT::FLOAT32);
  addField("y", PointFieldT::FLOAT32);
  addField("z", PointFieldT::FLOAT32);
  for (const auto& l : float_layers) {
    addField(l, PointFieldT::FLOAT32);
  }
  if (has_color) {
    addField("rgb", PointFieldT::FLOAT32);
  }

  const uint32_t point_step = offset;

  // Prepare layer data pointers
  const float* elev_data = elev.data();
  std::vector<const float*> float_ptrs;
  float_ptrs.reserve(float_layers.size());
  for (const auto& l : float_layers) {
    float_ptrs.push_back(map.get(l).data());
  }
  const float* color_data =
      has_color ? map.get(layer::color).data() : nullptr;

  // Count valid points in submap region
  size_t valid_count = 0;
  for (Eigen::Index j = 0; j < sub_cols; ++j) {
    const Eigen::Index base = buf_col[j] * rows;
    for (Eigen::Index i = 0; i < sub_rows; ++i) {
      if (std::isfinite(elev_data[base + buf_row[i]])) ++valid_count;
    }
  }

  // Fill message metadata
  msg.header.stamp = stamp;
  msg.header.frame_id = map.getFrameId();
  msg.height = 1;
  msg.width = static_cast<uint32_t>(valid_count);
  msg.point_step = point_step;
  msg.row_step = static_cast<uint32_t>(valid_count) * point_step;
  msg.is_dense = true;
  msg.is_bigendian = false;
  msg.data.resize(valid_count * point_step);

  // Fill point data (column-major for cache efficiency)
  uint8_t* out = msg.data.data();
  for (Eigen::Index j = 0; j < sub_cols; ++j) {
    const float y = col_y[j];
    const Eigen::Index base = buf_col[j] * rows;
    for (Eigen::Index i = 0; i < sub_rows; ++i) {
      const Eigen::Index idx = base + buf_row[i];
      const float z = elev_data[idx];
      if (!std::isfinite(z)) continue;

      const float x = row_x[i];
      std::memcpy(out, &x, 4);
      out += 4;
      std::memcpy(out, &y, 4);
      out += 4;
      std::memcpy(out, &z, 4);
      out += 4;

      for (const float* ptr : float_ptrs) {
        const float val = ptr[idx];
        std::memcpy(out, &val, 4);
        out += 4;
      }

      if (color_data) {
        std::memcpy(out, &color_data[idx], 4);
        out += 4;
      }
    }
  }

  return msg;
}

/// Full-map convenience overload.
template <typename PointCloud2T, typename PointFieldT, typename TimeT>
PointCloud2T toPointCloud2Impl(const ElevationMap& map, const TimeT& stamp,
                               const char* elevation_layer) {
  return toPointCloud2Impl<PointCloud2T, PointFieldT>(
      map, stamp, elevation_layer, map.getStartIndex(), map.getSize());
}

// ── fillGridMapMsg ───────────────────────────────────────────────────────────
//
// Fills GridMap message geometry, layers, data, and start indices.
// Does NOT set header (caller handles it — ROS1: msg.info.header, ROS2: msg.header).
//
template <typename GridMapMsgT, typename Float32MultiArrayT>
void fillGridMapMsg(GridMapMsgT& msg, const ElevationMap& map) {
  // Geometry
  msg.info.resolution = map.getResolution();
  msg.info.length_x = map.getLength().x();
  msg.info.length_y = map.getLength().y();
  msg.info.pose.position.x = map.getPosition().x();
  msg.info.pose.position.y = map.getPosition().y();
  msg.info.pose.position.z = 0.0;
  msg.info.pose.orientation.w = 1.0;
  msg.info.pose.orientation.x = 0.0;
  msg.info.pose.orientation.y = 0.0;
  msg.info.pose.orientation.z = 0.0;

  // Layers (skip internal layers)
  for (const auto& l : map.getLayers()) {
    if (!layer::isInternal(l)) msg.layers.push_back(l);
  }
  msg.basic_layers = map.getBasicLayers();

  // Data
  for (const auto& layer_name : msg.layers) {
    Float32MultiArrayT data_array;

    // Layout
    data_array.layout.dim.resize(2);
    data_array.layout.dim[0].label = "column_index";
    data_array.layout.dim[0].size = map.getSize()(0);
    data_array.layout.dim[0].stride = map.getSize()(0) * map.getSize()(1);
    data_array.layout.dim[1].label = "row_index";
    data_array.layout.dim[1].size = map.getSize()(1);
    data_array.layout.dim[1].stride = map.getSize()(1);

    // Copy data (Eigen column-major to row-major)
    const auto& layer_data = map.get(layer_name);
    data_array.data.resize(layer_data.size());

    size_t idx = 0;
    for (Eigen::Index col = 0; col < layer_data.cols(); ++col) {
      for (Eigen::Index row = 0; row < layer_data.rows(); ++row) {
        data_array.data[idx++] = layer_data(row, col);
      }
    }

    msg.data.push_back(data_array);
  }

  // Circular buffer start indices
  msg.outer_start_index = map.getStartIndex()(0);
  msg.inner_start_index = map.getStartIndex()(1);
}

// ── toMarkerImpl ─────────────────────────────────────────────────────────────
//
// Creates a LINE_STRIP marker showing the map boundary rectangle.
//
template <typename MarkerT, typename PointT, typename TimeT>
MarkerT toMarkerImpl(const ElevationMap& map, const TimeT& stamp) {
  MarkerT marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = map.getFrameId();
  marker.ns = "fastdem";
  marker.id = 0;
  marker.type = MarkerT::LINE_STRIP;
  marker.action = MarkerT::ADD;
  marker.scale.x = 0.01;
  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;
  marker.pose.orientation.w = 1.0;

  const double cx = map.getPosition().x();
  const double cy = map.getPosition().y();
  const double hx = map.getLength().x() / 2.0;
  const double hy = map.getLength().y() / 2.0;

  PointT p;
  p.z = 0.0;
  const double corners[][2] = {{cx - hx, cy - hy},
                                {cx + hx, cy - hy},
                                {cx + hx, cy + hy},
                                {cx - hx, cy + hy},
                                {cx - hx, cy - hy}};
  for (const auto& c : corners) {
    p.x = c[0];
    p.y = c[1];
    marker.points.push_back(p);
  }

  return marker;
}

}  // namespace fastdem::detail

#endif  // FASTDEM_BRIDGE_ROS_IMPL_HPP
