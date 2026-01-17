// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Implementation details for ROS 1 bridge.
// Do not include this file directly; include <nanopcl/bridge/ros1.hpp>

#ifndef NANOPCL_BRIDGE_IMPL_ROS1_IMPL_HPP
#define NANOPCL_BRIDGE_IMPL_ROS1_IMPL_HPP

#include <cmath>
#include <cstring>

namespace npcl {

namespace detail {

// =============================================================================
// Field Offset Parser
// =============================================================================

struct Ros1FieldOffsets {
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

  bool isXYZIContiguous(size_t point_step) const {
    return x == 0 && y == 4 && z == 8 && intensity == 12 &&
           intensity_type == sensor_msgs::PointField::FLOAT32 &&
           point_step >= 16;
  }
};

inline Ros1FieldOffsets parseRos1Fields(
    const std::vector<sensor_msgs::PointField>& fields) {
  Ros1FieldOffsets offsets;

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

inline float extractRos1Intensity(const uint8_t* ptr, int offset,
                                  uint8_t type) {
  using PF = sensor_msgs::PointField;
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

inline uint16_t extractRos1Ring(const uint8_t* ptr, int offset, uint8_t type) {
  using PF = sensor_msgs::PointField;
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
// Convert<sensor_msgs::PointCloud2> Implementation
// =============================================================================

template <>
struct Convert<sensor_msgs::PointCloud2> {
  static PointCloud from(const sensor_msgs::PointCloud2& msg) {
    const size_t num_points = static_cast<size_t>(msg.width) * msg.height;
    PointCloud cloud(num_points);

    // Set metadata
    cloud.setFrameId(msg.header.frame_id);
    cloud.setTimestamp(msg.header.stamp.toNSec());

    if (num_points == 0) {
      return cloud;
    }

    // Parse field offsets
    const auto offsets = detail::parseRos1Fields(msg.fields);
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

    // Fast path: XYZI-only with contiguous layout (most common LiDAR format)
    const bool xyzi_only = offsets.hasIntensity() && !offsets.hasTime() &&
                           !offsets.hasRing() && !offsets.hasColor() &&
                           !offsets.hasNormal() && !offsets.hasLabel();

    if (msg.is_dense && xyzi_only && offsets.isXYZIContiguous(point_step) &&
        point_step == 16) {
      // Optimal: XYZI contiguous, dense - single memcpy
      cloud.xyz().resize(num_points);
      cloud.intensity().resize(num_points);
      // Interleaved XYZI -> separate XYZ and I arrays
      const float* src = reinterpret_cast<const float*>(data_ptr);
      float* xyz_dst = reinterpret_cast<float*>(cloud.xyz().data());
      float* i_dst = cloud.intensity().data();
      for (size_t i = 0; i < num_points; ++i) {
        xyz_dst[i * 3 + 0] = src[i * 4 + 0];
        xyz_dst[i * 3 + 1] = src[i * 4 + 1];
        xyz_dst[i * 3 + 2] = src[i * 4 + 2];
        i_dst[i] = src[i * 4 + 3];
      }
      return cloud;
    }

    // Fast path: XYZ-only with contiguous layout
    const bool xyz_only = !offsets.hasIntensity() && !offsets.hasTime() &&
                          !offsets.hasRing() && !offsets.hasColor() &&
                          !offsets.hasNormal() && !offsets.hasLabel();

    if (msg.is_dense && xyz_only && offsets.isXYZContiguous(point_step) &&
        point_step == 12) {
      cloud.xyz().resize(num_points);
      std::memcpy(cloud.xyz().data(), data_ptr, num_points * sizeof(Point));
      return cloud;
    }

    // General path: optimized for dense clouds (most common case)
    if (msg.is_dense) {
      // Pre-allocate all arrays
      cloud.xyz().resize(num_points);
      if (offsets.hasIntensity()) cloud.intensity().resize(num_points);
      if (offsets.hasTime()) cloud.time().resize(num_points);
      if (offsets.hasRing()) cloud.ring().resize(num_points);
      if (offsets.hasColor()) cloud.color().resize(num_points);
      if (offsets.hasNormal()) cloud.normal().resize(num_points);
      if (offsets.hasLabel()) cloud.label().resize(num_points);

      // Get direct pointers (avoid per-element bounds checking)
      Point* xyz_ptr = cloud.xyz().data();
      float* i_ptr = offsets.hasIntensity() ? cloud.intensity().data() : nullptr;
      float* t_ptr = offsets.hasTime() ? cloud.time().data() : nullptr;
      uint16_t* r_ptr = offsets.hasRing() ? cloud.ring().data() : nullptr;
      Color* c_ptr = offsets.hasColor() ? cloud.color().data() : nullptr;
      Eigen::Vector3f* n_ptr = offsets.hasNormal() ? cloud.normal().data() : nullptr;
      Label* l_ptr = offsets.hasLabel() ? cloud.label().data() : nullptr;

      // Cache offsets locally
      const int x_off = offsets.x, y_off = offsets.y, z_off = offsets.z;
      const int int_off = offsets.intensity, time_off = offsets.time;
      const int ring_off = offsets.ring;
      const uint8_t int_type = offsets.intensity_type;
      const uint8_t ring_type = offsets.ring_type;

      for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* pt = data_ptr + i * point_step;

        xyz_ptr[i] = Point(*reinterpret_cast<const float*>(pt + x_off),
                           *reinterpret_cast<const float*>(pt + y_off),
                           *reinterpret_cast<const float*>(pt + z_off));

        if (i_ptr) i_ptr[i] = detail::extractRos1Intensity(pt, int_off, int_type);
        if (t_ptr) t_ptr[i] = *reinterpret_cast<const float*>(pt + time_off);
        if (r_ptr) r_ptr[i] = detail::extractRos1Ring(pt, ring_off, ring_type);

        if (c_ptr) {
          if (offsets.hasRGBSeparate()) {
            c_ptr[i] = Color{*(pt + offsets.r), *(pt + offsets.g), *(pt + offsets.b)};
          } else {
            const uint32_t rgb = *reinterpret_cast<const uint32_t*>(pt + offsets.rgb);
            c_ptr[i] = Color{static_cast<uint8_t>((rgb >> 16) & 0xFF),
                             static_cast<uint8_t>((rgb >> 8) & 0xFF),
                             static_cast<uint8_t>(rgb & 0xFF)};
          }
        }

        if (n_ptr) {
          n_ptr[i] = Eigen::Vector3f(
              *reinterpret_cast<const float*>(pt + offsets.normal_x),
              *reinterpret_cast<const float*>(pt + offsets.normal_y),
              *reinterpret_cast<const float*>(pt + offsets.normal_z));
        }

        if (l_ptr) {
          l_ptr[i] = Label(*reinterpret_cast<const uint32_t*>(pt + offsets.label));
        }
      }
    } else {
      // Sparse path: skip invalid points
      for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* pt_ptr = data_ptr + i * point_step;

        const float x = *reinterpret_cast<const float*>(pt_ptr + offsets.x);
        const float y = *reinterpret_cast<const float*>(pt_ptr + offsets.y);
        const float z = *reinterpret_cast<const float*>(pt_ptr + offsets.z);

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        cloud.xyz().emplace_back(x, y, z);

        if (offsets.hasIntensity()) {
          float intensity = detail::extractRos1Intensity(
              pt_ptr, offsets.intensity, offsets.intensity_type);
          cloud.intensity().push_back(std::isfinite(intensity) ? intensity : 0.0f);
        }
        if (offsets.hasTime()) {
          float t = *reinterpret_cast<const float*>(pt_ptr + offsets.time);
          cloud.time().push_back(std::isfinite(t) ? t : 0.0f);
        }
        if (offsets.hasRing()) {
          cloud.ring().push_back(
              detail::extractRos1Ring(pt_ptr, offsets.ring, offsets.ring_type));
        }
        if (offsets.hasRGBSeparate()) {
          cloud.color().push_back(Color{*(pt_ptr + offsets.r),
                                        *(pt_ptr + offsets.g),
                                        *(pt_ptr + offsets.b)});
        } else if (offsets.hasRGBPacked()) {
          const uint32_t rgb = *reinterpret_cast<const uint32_t*>(pt_ptr + offsets.rgb);
          cloud.color().push_back(
              Color{static_cast<uint8_t>((rgb >> 16) & 0xFF),
                    static_cast<uint8_t>((rgb >> 8) & 0xFF),
                    static_cast<uint8_t>(rgb & 0xFF)});
        }
        if (offsets.hasNormal()) {
          cloud.normal().emplace_back(
              *reinterpret_cast<const float*>(pt_ptr + offsets.normal_x),
              *reinterpret_cast<const float*>(pt_ptr + offsets.normal_y),
              *reinterpret_cast<const float*>(pt_ptr + offsets.normal_z));
        }
        if (offsets.hasLabel()) {
          cloud.label().push_back(
              Label(*reinterpret_cast<const uint32_t*>(pt_ptr + offsets.label)));
        }
      }
    }

    return cloud;
  }

  static sensor_msgs::PointCloud2 to(const PointCloud& cloud) {
    sensor_msgs::PointCloud2 msg;

    // Set metadata
    msg.header.frame_id = cloud.frameId();
    msg.header.stamp.fromNSec(cloud.timestamp());

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
    using PF = sensor_msgs::PointField;
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

    // Check fast paths
    const bool xyz_only = !cloud.hasIntensity() && !cloud.hasTime() &&
                          !cloud.hasRing() && !cloud.hasColor() &&
                          !cloud.hasNormal() && !cloud.hasLabel();
    const bool xyzi_only = cloud.hasIntensity() && !cloud.hasTime() &&
                           !cloud.hasRing() && !cloud.hasColor() &&
                           !cloud.hasNormal() && !cloud.hasLabel();

    if (xyz_only && msg.point_step == 12) {
      // Fast path: XYZ only - direct memcpy
      std::memcpy(data_ptr, cloud.xyz().data(), cloud.size() * sizeof(Point));
    } else if (xyzi_only && msg.point_step == 16) {
      // Fast path: XYZI only - optimized interleave
      const float* xyz_src = reinterpret_cast<const float*>(cloud.xyz().data());
      const float* i_src = cloud.intensity().data();
      float* dst = reinterpret_cast<float*>(data_ptr);
      for (size_t i = 0; i < cloud.size(); ++i) {
        dst[i * 4 + 0] = xyz_src[i * 3 + 0];
        dst[i * 4 + 1] = xyz_src[i * 3 + 1];
        dst[i * 4 + 2] = xyz_src[i * 3 + 2];
        dst[i * 4 + 3] = i_src[i];
      }
    } else {
      // General path: optimized with direct pointer access
      const size_t n = cloud.size();
      const size_t step = msg.point_step;

      // Get source pointers
      const Point* xyz_src = cloud.xyz().data();
      const float* i_src = cloud.hasIntensity() ? cloud.intensity().data() : nullptr;
      const float* t_src = cloud.hasTime() ? cloud.time().data() : nullptr;
      const uint16_t* r_src = cloud.hasRing() ? cloud.ring().data() : nullptr;
      const Color* c_src = cloud.hasColor() ? cloud.color().data() : nullptr;
      const Eigen::Vector3f* n_src = cloud.hasNormal() ? cloud.normal().data() : nullptr;
      const Label* l_src = cloud.hasLabel() ? cloud.label().data() : nullptr;

      for (size_t i = 0; i < n; ++i) {
        uint8_t* pt = data_ptr + i * step;

        const auto& p = xyz_src[i];
        *reinterpret_cast<float*>(pt + 0) = p.x();
        *reinterpret_cast<float*>(pt + 4) = p.y();
        *reinterpret_cast<float*>(pt + 8) = p.z();

        if (i_src) *reinterpret_cast<float*>(pt + intensity_off) = i_src[i];
        if (t_src) *reinterpret_cast<float*>(pt + time_off) = t_src[i];
        if (r_src) *reinterpret_cast<uint16_t*>(pt + ring_off) = r_src[i];
        if (c_src) {
          *(pt + r_off) = c_src[i][0];
          *(pt + g_off) = c_src[i][1];
          *(pt + b_off) = c_src[i][2];
        }
        if (n_src) {
          *reinterpret_cast<float*>(pt + nx_off) = n_src[i].x();
          *reinterpret_cast<float*>(pt + ny_off) = n_src[i].y();
          *reinterpret_cast<float*>(pt + nz_off) = n_src[i].z();
        }
        if (l_src) {
          *reinterpret_cast<uint32_t*>(pt + label_off) = l_src[i].val;
        }
      }
    }

    return msg;
  }
};

// =============================================================================
// Validation
// =============================================================================

[[nodiscard]] inline bool validate(const sensor_msgs::PointCloud2& msg) {
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

[[nodiscard]] inline Transform from(const geometry_msgs::Transform& msg,
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
    const geometry_msgs::TransformStamped& msg) {
  return from(msg.transform, msg.header.frame_id, msg.child_frame_id);
}

[[nodiscard]] inline geometry_msgs::Transform toTransformMsg(
    const Transform& tf) {
  geometry_msgs::Transform msg;

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

[[nodiscard]] inline geometry_msgs::TransformStamped toTransformStampedMsg(
    const Transform& tf, const ros::Time& stamp) {
  geometry_msgs::TransformStamped msg;

  msg.header.stamp = stamp;
  msg.header.frame_id = tf.parentFrame();
  msg.child_frame_id = tf.childFrame();
  msg.transform = toTransformMsg(tf);

  return msg;
}

}  // namespace npcl

#endif  // NANOPCL_BRIDGE_IMPL_ROS1_IMPL_HPP
