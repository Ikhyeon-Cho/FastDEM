// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// ROS bridge implementation details.
// Do not include this file directly; include <nanopcl/bridge/ros1.hpp> or
// <nanopcl/bridge/ros2.hpp>

#ifndef NANOPCL_BRIDGE_ROS_IMPL_HPP
#define NANOPCL_BRIDGE_ROS_IMPL_HPP

#include <cmath>
#include <cstdint>
#include <cstring>

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {

namespace detail {

// PointField datatype constants (same for ROS1 and ROS2)
struct PointFieldTypes {
  static constexpr uint8_t INT8 = 1;
  static constexpr uint8_t UINT8 = 2;
  static constexpr uint8_t INT16 = 3;
  static constexpr uint8_t UINT16 = 4;
  static constexpr uint8_t INT32 = 5;
  static constexpr uint8_t UINT32 = 6;
  static constexpr uint8_t FLOAT32 = 7;
  static constexpr uint8_t FLOAT64 = 8;
};

/// @brief Field offsets parsed from PointCloud2 message
struct FieldOffsets {
  // XYZ (required)
  int x = -1;
  int y = -1;
  int z = -1;

  // Optional channels
  int intensity = -1;
  int ring = -1;
  int time = -1;
  int rgb = -1;
  int label = -1;
  int normal_x = -1;
  int normal_y = -1;
  int normal_z = -1;

  // Datatype info for type conversion
  uint8_t intensity_type = 0;
  uint8_t ring_type = 0;
  uint8_t time_type = 0;
  uint8_t label_type = 0;

  bool has_xyz() const { return x >= 0 && y >= 0 && z >= 0; }
  bool has_intensity() const { return intensity >= 0; }
  bool has_ring() const { return ring >= 0; }
  bool has_time() const { return time >= 0; }
  bool has_rgb() const { return rgb >= 0; }
  bool has_label() const { return label >= 0; }
  bool has_normal() const {
    return normal_x >= 0 && normal_y >= 0 && normal_z >= 0;
  }

  /// @brief Parse field offsets from PointCloud2 fields
  template <typename FieldT>
  static FieldOffsets parse(const std::vector<FieldT>& fields) {
    FieldOffsets offsets;
    for (const auto& field : fields) {
      if (field.name == "x") {
        offsets.x = static_cast<int>(field.offset);
      } else if (field.name == "y") {
        offsets.y = static_cast<int>(field.offset);
      } else if (field.name == "z") {
        offsets.z = static_cast<int>(field.offset);
      } else if (field.name == "intensity") {
        offsets.intensity = static_cast<int>(field.offset);
        offsets.intensity_type = field.datatype;
      } else if (field.name == "ring") {
        offsets.ring = static_cast<int>(field.offset);
        offsets.ring_type = field.datatype;
      } else if (field.name == "t" || field.name == "time" ||
                 field.name == "timestamp") {
        offsets.time = static_cast<int>(field.offset);
        offsets.time_type = field.datatype;
      } else if (field.name == "rgb" || field.name == "rgba") {
        offsets.rgb = static_cast<int>(field.offset);
      } else if (field.name == "label") {
        offsets.label = static_cast<int>(field.offset);
        offsets.label_type = field.datatype;
      } else if (field.name == "normal_x") {
        offsets.normal_x = static_cast<int>(field.offset);
      } else if (field.name == "normal_y") {
        offsets.normal_y = static_cast<int>(field.offset);
      } else if (field.name == "normal_z") {
        offsets.normal_z = static_cast<int>(field.offset);
      }
    }
    return offsets;
  }
};

/// @brief Read intensity value with type conversion
inline float readIntensity(const uint8_t* ptr, int offset, uint8_t datatype) {
  const uint8_t* p = ptr + offset;
  switch (datatype) {
  case PointFieldTypes::UINT8:
    return static_cast<float>(*p);
  case PointFieldTypes::UINT16:
    return static_cast<float>(*reinterpret_cast<const uint16_t*>(p));
  case PointFieldTypes::FLOAT32:
    return *reinterpret_cast<const float*>(p);
  case PointFieldTypes::FLOAT64:
    return static_cast<float>(*reinterpret_cast<const double*>(p));
  default:
    return 0.0f;
  }
}

/// @brief Read ring value with type conversion
inline uint16_t readRing(const uint8_t* ptr, int offset, uint8_t datatype) {
  const uint8_t* p = ptr + offset;
  switch (datatype) {
  case PointFieldTypes::UINT8:
    return static_cast<uint16_t>(*p);
  case PointFieldTypes::UINT16:
    return *reinterpret_cast<const uint16_t*>(p);
  case PointFieldTypes::INT32:
    return static_cast<uint16_t>(*reinterpret_cast<const int32_t*>(p));
  default:
    return 0;
  }
}

/// @brief Read time value with type conversion
inline float readTime(const uint8_t* ptr, int offset, uint8_t datatype) {
  const uint8_t* p = ptr + offset;
  switch (datatype) {
  case PointFieldTypes::FLOAT32:
    return *reinterpret_cast<const float*>(p);
  case PointFieldTypes::FLOAT64:
    return static_cast<float>(*reinterpret_cast<const double*>(p));
  case PointFieldTypes::UINT32:
    // Ouster-style nanoseconds offset
    return static_cast<float>(*reinterpret_cast<const uint32_t*>(p)) * 1e-9f;
  default:
    return 0.0f;
  }
}

/// @brief Read label value with type conversion
inline uint32_t readLabel(const uint8_t* ptr, int offset, uint8_t datatype) {
  const uint8_t* p = ptr + offset;
  switch (datatype) {
  case PointFieldTypes::UINT8:
    return static_cast<uint32_t>(*p);
  case PointFieldTypes::UINT16:
    return static_cast<uint32_t>(*reinterpret_cast<const uint16_t*>(p));
  case PointFieldTypes::UINT32:
    return *reinterpret_cast<const uint32_t*>(p);
  case PointFieldTypes::INT32:
    return static_cast<uint32_t>(*reinterpret_cast<const int32_t*>(p));
  default:
    return 0;
  }
}

/// @brief Read RGB from packed float (PCL convention)
inline Color readRgb(const uint8_t* ptr, int offset) {
  // RGB is stored as packed uint32 in a float field
  uint32_t rgb;
  std::memcpy(&rgb, ptr + offset, sizeof(uint32_t));
  return Color{static_cast<uint8_t>((rgb >> 16) & 0xFF),
               static_cast<uint8_t>((rgb >> 8) & 0xFF),
               static_cast<uint8_t>(rgb & 0xFF)};
}

/// @brief Convert PointCloud2 message to nanoPCL PointCloud (implementation)
template <typename PointCloud2T, typename PointFieldT>
PointCloud from_impl(const PointCloud2T& msg) {
  PointCloud cloud;

  const size_t num_points = static_cast<size_t>(msg.width) * msg.height;
  if (num_points == 0) {
    return cloud;
  }

  const auto offsets = FieldOffsets::parse<PointFieldT>(msg.fields);
  if (!offsets.has_xyz()) {
    return cloud;
  }

  // 1. Activate channels BEFORE adding points
  if (offsets.has_intensity())
    cloud.useIntensity();
  if (offsets.has_ring())
    cloud.useRing();
  if (offsets.has_time())
    cloud.useTime();
  if (offsets.has_rgb())
    cloud.useColor();
  if (offsets.has_label())
    cloud.useLabel();
  if (offsets.has_normal())
    cloud.useNormal();

  // 2. Reserve memory
  cloud.points().reserve(num_points);
  if (cloud.hasIntensity())
    cloud.intensities().reserve(num_points);
  if (cloud.hasRing())
    cloud.rings().reserve(num_points);
  if (cloud.hasTime())
    cloud.times().reserve(num_points);
  if (cloud.hasColor())
    cloud.colors().reserve(num_points);
  if (cloud.hasLabel())
    cloud.labels().reserve(num_points);
  if (cloud.hasNormal())
    cloud.normals().reserve(num_points);

  // 3. Set metadata
  cloud.setFrameId(msg.header.frame_id);

  // 4. Parse points
  const uint8_t* data_ptr = msg.data.data();
  const size_t point_step = msg.point_step;

  for (size_t i = 0; i < num_points; ++i) {
    const uint8_t* pt = data_ptr + i * point_step;

    const float x = *reinterpret_cast<const float*>(pt + offsets.x);
    const float y = *reinterpret_cast<const float*>(pt + offsets.y);
    const float z = *reinterpret_cast<const float*>(pt + offsets.z);

    // Skip invalid points
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    cloud.points().emplace_back(x, y, z, 1.0f);

    if (cloud.hasIntensity()) {
      cloud.intensities().push_back(
          readIntensity(pt, offsets.intensity, offsets.intensity_type));
    }
    if (cloud.hasRing()) {
      cloud.rings().push_back(readRing(pt, offsets.ring, offsets.ring_type));
    }
    if (cloud.hasTime()) {
      cloud.times().push_back(readTime(pt, offsets.time, offsets.time_type));
    }
    if (cloud.hasColor()) {
      cloud.colors().push_back(readRgb(pt, offsets.rgb));
    }
    if (cloud.hasLabel()) {
      cloud.labels().push_back(
          Label(readLabel(pt, offsets.label, offsets.label_type)));
    }
    if (cloud.hasNormal()) {
      const float nx = *reinterpret_cast<const float*>(pt + offsets.normal_x);
      const float ny = *reinterpret_cast<const float*>(pt + offsets.normal_y);
      const float nz = *reinterpret_cast<const float*>(pt + offsets.normal_z);
      cloud.normals().emplace_back(nx, ny, nz, 0.0f);
    }
  }

  return cloud;
}

/// @brief Convert nanoPCL PointCloud to PointCloud2 message (implementation)
template <typename PointCloud2T, typename PointFieldT, typename TimeT>
PointCloud2T to_impl(const PointCloud& cloud, const std::string& frame_id, const TimeT& stamp) {
  PointCloud2T msg;

  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.height = 1;
  msg.width = static_cast<uint32_t>(cloud.size());
  msg.is_bigendian = false;
  msg.is_dense = true;

  if (cloud.empty()) {
    msg.point_step = 0;
    msg.row_step = 0;
    return msg;
  }

  // Build fields based on active channels
  PointFieldT field;
  field.count = 1;
  field.datatype = PointFieldTypes::FLOAT32;

  uint32_t offset = 0;

  // XYZ (always present)
  field.name = "x";
  field.offset = offset;
  msg.fields.push_back(field);
  offset += sizeof(float);

  field.name = "y";
  field.offset = offset;
  msg.fields.push_back(field);
  offset += sizeof(float);

  field.name = "z";
  field.offset = offset;
  msg.fields.push_back(field);
  offset += sizeof(float);

  // Optional channels
  const int intensity_offset = cloud.hasIntensity() ? static_cast<int>(offset) : -1;
  if (cloud.hasIntensity()) {
    field.name = "intensity";
    field.offset = offset;
    msg.fields.push_back(field);
    offset += sizeof(float);
  }

  const int ring_offset = cloud.hasRing() ? static_cast<int>(offset) : -1;
  if (cloud.hasRing()) {
    field.name = "ring";
    field.offset = offset;
    field.datatype = PointFieldTypes::UINT16;
    msg.fields.push_back(field);
    field.datatype = PointFieldTypes::FLOAT32;
    offset += sizeof(uint16_t);
  }

  const int time_offset = cloud.hasTime() ? static_cast<int>(offset) : -1;
  if (cloud.hasTime()) {
    field.name = "time";
    field.offset = offset;
    msg.fields.push_back(field);
    offset += sizeof(float);
  }

  const int rgb_offset = cloud.hasColor() ? static_cast<int>(offset) : -1;
  if (cloud.hasColor()) {
    field.name = "rgb";
    field.offset = offset;
    msg.fields.push_back(field);
    offset += sizeof(float);
  }

  const int label_offset = cloud.hasLabel() ? static_cast<int>(offset) : -1;
  if (cloud.hasLabel()) {
    field.name = "label";
    field.offset = offset;
    field.datatype = PointFieldTypes::UINT32;
    msg.fields.push_back(field);
    field.datatype = PointFieldTypes::FLOAT32;
    offset += sizeof(uint32_t);
  }

  const int normal_x_offset = cloud.hasNormal() ? static_cast<int>(offset) : -1;
  if (cloud.hasNormal()) {
    field.name = "normal_x";
    field.offset = offset;
    msg.fields.push_back(field);
    offset += sizeof(float);

    field.name = "normal_y";
    field.offset = offset;
    msg.fields.push_back(field);
    offset += sizeof(float);

    field.name = "normal_z";
    field.offset = offset;
    msg.fields.push_back(field);
    offset += sizeof(float);
  }

  msg.point_step = offset;
  msg.row_step = msg.point_step * msg.width;

  // Fill data buffer
  msg.data.resize(msg.row_step);
  uint8_t* data_ptr = msg.data.data();

  for (size_t i = 0; i < cloud.size(); ++i) {
    uint8_t* pt = data_ptr + i * msg.point_step;

    // XYZ
    const auto& p = cloud[i];
    *reinterpret_cast<float*>(pt + 0) = p.x();
    *reinterpret_cast<float*>(pt + 4) = p.y();
    *reinterpret_cast<float*>(pt + 8) = p.z();

    // Optional channels
    if (intensity_offset >= 0) {
      *reinterpret_cast<float*>(pt + intensity_offset) = cloud.intensity(i);
    }
    if (ring_offset >= 0) {
      *reinterpret_cast<uint16_t*>(pt + ring_offset) = cloud.ring(i);
    }
    if (time_offset >= 0) {
      *reinterpret_cast<float*>(pt + time_offset) = cloud.time(i);
    }
    if (rgb_offset >= 0) {
      const auto& c = cloud.color(i);
      uint32_t rgb = (static_cast<uint32_t>(c.r) << 16) |
                     (static_cast<uint32_t>(c.g) << 8) |
                     static_cast<uint32_t>(c.b);
      std::memcpy(pt + rgb_offset, &rgb, sizeof(uint32_t));
    }
    if (label_offset >= 0) {
      *reinterpret_cast<uint32_t*>(pt + label_offset) = cloud.label(i).val;
    }
    if (normal_x_offset >= 0) {
      const auto& n = cloud.normals()[i];
      *reinterpret_cast<float*>(pt + normal_x_offset) = n.x();
      *reinterpret_cast<float*>(pt + normal_x_offset + 4) = n.y();
      *reinterpret_cast<float*>(pt + normal_x_offset + 8) = n.z();
    }
  }

  return msg;
}

} // namespace detail

} // namespace nanopcl

#endif // NANOPCL_BRIDGE_ROS_IMPL_HPP
