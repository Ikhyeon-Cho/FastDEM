// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Implementation details for ROS 2 bridge.
// Do not include this file directly; include <nanopcl/bridge/ros2.hpp>

#ifndef NANOPCL_BRIDGE_IMPL_ROS2_IMPL_HPP
#define NANOPCL_BRIDGE_IMPL_ROS2_IMPL_HPP

#include <cmath>
#include <cstring>

namespace npcl {

namespace detail {

// =============================================================================
// Field Offset Parser
// =============================================================================

struct Ros2FieldOffsets {
  int x = -1, y = -1, z = -1;
  int intensity = -1;
  int time = -1;
  int ring = -1;
  int r = -1, g = -1, b = -1;
  int rgb = -1;
  int normal_x = -1, normal_y = -1, normal_z = -1;
  int label = -1;

  uint8_t intensity_type = 0;
  uint8_t ring_type = 0;

  bool hasXYZ() const { return x >= 0 && y >= 0 && z >= 0; }
  bool hasIntensity() const { return intensity >= 0; }
  bool hasTime() const { return time >= 0; }
  bool hasRing() const { return ring >= 0; }
  bool hasRGBSeparate() const { return r >= 0 && g >= 0 && b >= 0; }
  bool hasRGBPacked() const { return rgb >= 0; }
  bool hasColor() const { return hasRGBSeparate() || hasRGBPacked(); }
  bool hasNormal() const {
    return normal_x >= 0 && normal_y >= 0 && normal_z >= 0;
  }
  bool hasLabel() const { return label >= 0; }

  bool isXYZContiguous(size_t point_step) const {
    return x == 0 && y == 4 && z == 8 && point_step >= 12;
  }
};

inline Ros2FieldOffsets parseRos2Fields(
    const std::vector<sensor_msgs::msg::PointField>& fields) {
  Ros2FieldOffsets offsets;

  for (const auto& field : fields) {
    if (field.name == "x")
      offsets.x = field.offset;
    else if (field.name == "y")
      offsets.y = field.offset;
    else if (field.name == "z")
      offsets.z = field.offset;
    else if (field.name == "intensity") {
      offsets.intensity = field.offset;
      offsets.intensity_type = field.datatype;
    } else if (field.name == "t" || field.name == "time" ||
               field.name == "timestamp" || field.name == "time_stamp")
      offsets.time = field.offset;
    else if (field.name == "ring") {
      offsets.ring = field.offset;
      offsets.ring_type = field.datatype;
    } else if (field.name == "r" || field.name == "red")
      offsets.r = field.offset;
    else if (field.name == "g" || field.name == "green")
      offsets.g = field.offset;
    else if (field.name == "b" || field.name == "blue")
      offsets.b = field.offset;
    else if (field.name == "rgb" || field.name == "rgba")
      offsets.rgb = field.offset;
    else if (field.name == "normal_x")
      offsets.normal_x = field.offset;
    else if (field.name == "normal_y")
      offsets.normal_y = field.offset;
    else if (field.name == "normal_z")
      offsets.normal_z = field.offset;
    else if (field.name == "label")
      offsets.label = field.offset;
  }

  return offsets;
}

// =============================================================================
// Field Extractors
// =============================================================================

inline float extractRos2Intensity(const uint8_t* ptr, int offset,
                                  uint8_t type) {
  using PF = sensor_msgs::msg::PointField;
  switch (type) {
    case PF::FLOAT32:
      return *reinterpret_cast<const float*>(ptr + offset);
    case PF::FLOAT64:
      return static_cast<float>(*reinterpret_cast<const double*>(ptr + offset));
    case PF::UINT8:
      return static_cast<float>(*(ptr + offset));
    case PF::UINT16:
      return static_cast<float>(
          *reinterpret_cast<const uint16_t*>(ptr + offset));
    case PF::UINT32:
      return static_cast<float>(
          *reinterpret_cast<const uint32_t*>(ptr + offset));
    case PF::INT8:
      return static_cast<float>(*reinterpret_cast<const int8_t*>(ptr + offset));
    case PF::INT16:
      return static_cast<float>(
          *reinterpret_cast<const int16_t*>(ptr + offset));
    case PF::INT32:
      return static_cast<float>(
          *reinterpret_cast<const int32_t*>(ptr + offset));
    default:
      return 0.0f;
  }
}

inline uint16_t extractRos2Ring(const uint8_t* ptr, int offset, uint8_t type) {
  using PF = sensor_msgs::msg::PointField;
  switch (type) {
    case PF::UINT8:
      return static_cast<uint16_t>(*(ptr + offset));
    case PF::UINT16:
      return *reinterpret_cast<const uint16_t*>(ptr + offset);
    case PF::UINT32:
      return static_cast<uint16_t>(
          *reinterpret_cast<const uint32_t*>(ptr + offset));
    case PF::INT8:
      return static_cast<uint16_t>(
          *reinterpret_cast<const int8_t*>(ptr + offset));
    case PF::INT16:
      return static_cast<uint16_t>(
          *reinterpret_cast<const int16_t*>(ptr + offset));
    case PF::INT32:
      return static_cast<uint16_t>(
          *reinterpret_cast<const int32_t*>(ptr + offset));
    default:
      return 0;
  }
}

}  // namespace detail

// =============================================================================
// Convert<sensor_msgs::msg::PointCloud2> Implementation
// =============================================================================

template <>
struct Convert<sensor_msgs::msg::PointCloud2> {
  static PointCloud from(const sensor_msgs::msg::PointCloud2& msg) {
    const size_t num_points = static_cast<size_t>(msg.width) * msg.height;
    PointCloud cloud(num_points);

    // Set metadata
    cloud.setFrameId(msg.header.frame_id);
    const uint64_t timestamp_ns =
        static_cast<uint64_t>(msg.header.stamp.sec) * 1'000'000'000ULL +
        msg.header.stamp.nanosec;
    cloud.setTimestamp(timestamp_ns);

    if (num_points == 0) {
      return cloud;
    }

    // Parse field offsets
    const auto offsets = detail::parseRos2Fields(msg.fields);
    if (!offsets.hasXYZ()) {
      return cloud;
    }

    const uint8_t* data_ptr = msg.data.data();
    const size_t point_step = msg.point_step;

    // Enable channels based on available fields
    if (offsets.hasIntensity()) cloud.enableIntensity();
    if (offsets.hasTime()) cloud.enableTime();
    if (offsets.hasRing()) cloud.enableRing();
    if (offsets.hasColor()) cloud.enableColor();
    if (offsets.hasNormal()) cloud.enableNormal();
    if (offsets.hasLabel()) cloud.enableLabel();

    // Reserve space
    cloud.xyz().reserve(num_points);
    if (offsets.hasIntensity()) cloud.intensity().reserve(num_points);
    if (offsets.hasTime()) cloud.time().reserve(num_points);
    if (offsets.hasRing()) cloud.ring().reserve(num_points);
    if (offsets.hasColor()) cloud.color().reserve(num_points);
    if (offsets.hasNormal()) cloud.normal().reserve(num_points);
    if (offsets.hasLabel()) cloud.label().reserve(num_points);

    // Fast path: memcpy for contiguous XYZ layout
    const bool use_fast_path =
        offsets.isXYZContiguous(point_step) && msg.is_dense;

    if (use_fast_path && point_step == 12) {
      cloud.xyz().resize(num_points);
      std::memcpy(cloud.xyz().data(), data_ptr, num_points * sizeof(Point));
    } else {
      for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* pt_ptr = data_ptr + i * point_step;

        const float x = *reinterpret_cast<const float*>(pt_ptr + offsets.x);
        const float y = *reinterpret_cast<const float*>(pt_ptr + offsets.y);
        const float z = *reinterpret_cast<const float*>(pt_ptr + offsets.z);

        if (!msg.is_dense &&
            (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))) {
          continue;
        }

        cloud.xyz().emplace_back(x, y, z);

        if (offsets.hasIntensity()) {
          float intensity = detail::extractRos2Intensity(
              pt_ptr, offsets.intensity, offsets.intensity_type);
          cloud.intensity().push_back(std::isfinite(intensity) ? intensity
                                                               : 0.0f);
        }

        if (offsets.hasTime()) {
          float t = *reinterpret_cast<const float*>(pt_ptr + offsets.time);
          cloud.time().push_back(std::isfinite(t) ? t : 0.0f);
        }

        if (offsets.hasRing()) {
          cloud.ring().push_back(
              detail::extractRos2Ring(pt_ptr, offsets.ring, offsets.ring_type));
        }

        if (offsets.hasRGBSeparate()) {
          cloud.color().push_back(Color{*(pt_ptr + offsets.r),
                                        *(pt_ptr + offsets.g),
                                        *(pt_ptr + offsets.b)});
        } else if (offsets.hasRGBPacked()) {
          const uint32_t rgb =
              *reinterpret_cast<const uint32_t*>(pt_ptr + offsets.rgb);
          cloud.color().push_back(
              Color{static_cast<uint8_t>((rgb >> 16) & 0xFF),
                    static_cast<uint8_t>((rgb >> 8) & 0xFF),
                    static_cast<uint8_t>(rgb & 0xFF)});
        }

        if (offsets.hasNormal()) {
          const float nx =
              *reinterpret_cast<const float*>(pt_ptr + offsets.normal_x);
          const float ny =
              *reinterpret_cast<const float*>(pt_ptr + offsets.normal_y);
          const float nz =
              *reinterpret_cast<const float*>(pt_ptr + offsets.normal_z);
          cloud.normal().emplace_back(nx, ny, nz);
        }

        if (offsets.hasLabel()) {
          cloud.label().push_back(Label(
              *reinterpret_cast<const uint32_t*>(pt_ptr + offsets.label)));
        }
      }
    }

    return cloud;
  }

  static sensor_msgs::msg::PointCloud2 to(const PointCloud& cloud) {
    sensor_msgs::msg::PointCloud2 msg;

    // Set metadata
    msg.header.frame_id = cloud.frameId();
    msg.header.stamp.sec =
        static_cast<int32_t>(cloud.timestamp() / 1'000'000'000ULL);
    msg.header.stamp.nanosec =
        static_cast<uint32_t>(cloud.timestamp() % 1'000'000'000ULL);

    // Set dimensions
    msg.height = 1;
    msg.width = static_cast<uint32_t>(cloud.size());
    msg.is_bigendian = false;
    msg.is_dense = true;

    if (cloud.empty()) {
      msg.point_step = 0;
      msg.row_step = 0;
      return msg;
    }

    // Build fields and calculate point_step
    using PF = sensor_msgs::msg::PointField;
    uint32_t offset = 0;

    auto addField = [&msg, &offset](const std::string& name, uint8_t datatype,
                                    uint32_t count = 1) -> uint32_t {
      PF field;
      field.name = name;
      field.offset = offset;
      field.datatype = datatype;
      field.count = count;
      msg.fields.push_back(field);

      uint32_t field_offset = offset;
      uint32_t size = 0;
      switch (datatype) {
        case PF::INT8:
        case PF::UINT8:
          size = 1;
          break;
        case PF::INT16:
        case PF::UINT16:
          size = 2;
          break;
        case PF::INT32:
        case PF::UINT32:
        case PF::FLOAT32:
          size = 4;
          break;
        case PF::FLOAT64:
          size = 8;
          break;
      }
      offset += size * count;
      return field_offset;
    };

    // XYZ (always present)
    addField("x", PF::FLOAT32);
    addField("y", PF::FLOAT32);
    addField("z", PF::FLOAT32);

    // Optional fields
    uint32_t intensity_off = 0, time_off = 0, ring_off = 0;
    uint32_t r_off = 0, g_off = 0, b_off = 0;
    uint32_t nx_off = 0, ny_off = 0, nz_off = 0;
    uint32_t label_off = 0;

    if (cloud.hasIntensity()) {
      intensity_off = addField("intensity", PF::FLOAT32);
    }
    if (cloud.hasTime()) {
      time_off = addField("t", PF::FLOAT32);
    }
    if (cloud.hasRing()) {
      ring_off = addField("ring", PF::UINT16);
    }
    if (cloud.hasColor()) {
      r_off = addField("r", PF::UINT8);
      g_off = addField("g", PF::UINT8);
      b_off = addField("b", PF::UINT8);
      offset += 1;  // Padding
    }
    if (cloud.hasNormal()) {
      nx_off = addField("normal_x", PF::FLOAT32);
      ny_off = addField("normal_y", PF::FLOAT32);
      nz_off = addField("normal_z", PF::FLOAT32);
    }
    if (cloud.hasLabel()) {
      label_off = addField("label", PF::UINT32);
    }

    msg.point_step = offset;
    msg.row_step = msg.point_step * msg.width;

    // Allocate data buffer
    msg.data.resize(msg.row_step);
    uint8_t* data_ptr = msg.data.data();

    // Check fast path
    const bool xyz_only = !cloud.hasIntensity() && !cloud.hasTime() &&
                          !cloud.hasRing() && !cloud.hasColor() &&
                          !cloud.hasNormal() && !cloud.hasLabel();

    if (xyz_only && msg.point_step == 12) {
      std::memcpy(data_ptr, cloud.xyz().data(), cloud.size() * sizeof(Point));
    } else {
      for (size_t i = 0; i < cloud.size(); ++i) {
        uint8_t* pt_ptr = data_ptr + i * msg.point_step;

        const auto& p = cloud[i];
        *reinterpret_cast<float*>(pt_ptr + 0) = p.x();
        *reinterpret_cast<float*>(pt_ptr + 4) = p.y();
        *reinterpret_cast<float*>(pt_ptr + 8) = p.z();

        if (cloud.hasIntensity()) {
          *reinterpret_cast<float*>(pt_ptr + intensity_off) =
              cloud.intensity()[i];
        }
        if (cloud.hasTime()) {
          *reinterpret_cast<float*>(pt_ptr + time_off) = cloud.time()[i];
        }
        if (cloud.hasRing()) {
          *reinterpret_cast<uint16_t*>(pt_ptr + ring_off) = cloud.ring()[i];
        }
        if (cloud.hasColor()) {
          const auto& c = cloud.color()[i];
          *(pt_ptr + r_off) = c[0];
          *(pt_ptr + g_off) = c[1];
          *(pt_ptr + b_off) = c[2];
        }
        if (cloud.hasNormal()) {
          const auto& n = cloud.normal()[i];
          *reinterpret_cast<float*>(pt_ptr + nx_off) = n.x();
          *reinterpret_cast<float*>(pt_ptr + ny_off) = n.y();
          *reinterpret_cast<float*>(pt_ptr + nz_off) = n.z();
        }
        if (cloud.hasLabel()) {
          *reinterpret_cast<uint32_t*>(pt_ptr + label_off) =
              cloud.label()[i].val;
        }
      }
    }

    return msg;
  }
};

// =============================================================================
// Validation
// =============================================================================

[[nodiscard]] inline bool validate(const sensor_msgs::msg::PointCloud2& msg) {
  if (msg.header.frame_id.empty()) {
    return false;
  }
  if (msg.width * msg.height == 0) {
    return false;
  }

  bool has_x = false, has_y = false, has_z = false;
  for (const auto& field : msg.fields) {
    if (field.name == "x") has_x = true;
    if (field.name == "y") has_y = true;
    if (field.name == "z") has_z = true;
  }

  return has_x && has_y && has_z;
}

// =============================================================================
// Transform Conversion
// =============================================================================

[[nodiscard]] inline Transform from(const geometry_msgs::msg::Transform& msg,
                                    const std::string& parent,
                                    const std::string& child) {
  Eigen::Vector3d trans(msg.translation.x, msg.translation.y,
                        msg.translation.z);
  Eigen::Quaterniond quat(msg.rotation.w, msg.rotation.x, msg.rotation.y,
                          msg.rotation.z);

  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.translate(trans);
  iso.rotate(quat);

  return Transform(parent, child, iso);
}

[[nodiscard]] inline Transform from(
    const geometry_msgs::msg::TransformStamped& msg) {
  return from(msg.transform, msg.header.frame_id, msg.child_frame_id);
}

[[nodiscard]] inline geometry_msgs::msg::Transform toTransformMsg(
    const Transform& tf) {
  geometry_msgs::msg::Transform msg;

  const auto& trans = tf.translation();
  msg.translation.x = trans.x();
  msg.translation.y = trans.y();
  msg.translation.z = trans.z();

  const auto quat = tf.quaternion();
  msg.rotation.x = quat.x();
  msg.rotation.y = quat.y();
  msg.rotation.z = quat.z();
  msg.rotation.w = quat.w();

  return msg;
}

[[nodiscard]] inline geometry_msgs::msg::TransformStamped toTransformStampedMsg(
    const Transform& tf, const builtin_interfaces::msg::Time& stamp) {
  geometry_msgs::msg::TransformStamped msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = tf.parentFrame();
  msg.child_frame_id = tf.childFrame();
  msg.transform = toTransformMsg(tf);

  return msg;
}

}  // namespace npcl

#endif  // NANOPCL_BRIDGE_IMPL_ROS2_IMPL_HPP
