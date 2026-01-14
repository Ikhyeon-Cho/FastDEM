// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Implementation details for PCD I/O.
// Do not include this file directly; include <nanopcl/io/pcd_io.hpp>

#ifndef NANOPCL_IO_IMPL_PCD_IO_IMPL_HPP
#define NANOPCL_IO_IMPL_PCD_IO_IMPL_HPP

#include <algorithm>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace npcl {
namespace io {
namespace detail {

/**
 * @brief Pack RGB into PCL-compatible float representation
 *
 * PCL stores RGB as a packed uint32 reinterpreted as float:
 * bits [23:16] = R, bits [15:8] = G, bits [7:0] = B
 */
inline float packRGBToFloat(uint8_t r, uint8_t g, uint8_t b) {
  uint32_t packed = (static_cast<uint32_t>(r) << 16) |
                    (static_cast<uint32_t>(g) << 8) | static_cast<uint32_t>(b);
  float result;
  std::memcpy(&result, &packed, sizeof(float));
  return result;
}

/**
 * @brief Unpack PCL-compatible float RGB to separate channels
 */
inline void unpackFloatToRGB(float rgb_float, uint8_t& r, uint8_t& g,
                             uint8_t& b) {
  uint32_t packed;
  std::memcpy(&packed, &rgb_float, sizeof(uint32_t));
  r = (packed >> 16) & 0xFF;
  g = (packed >> 8) & 0xFF;
  b = packed & 0xFF;
}

/**
 * @brief Field descriptor for dynamic PCD serialization
 */
struct FieldDesc {
  std::string name;
  int size;    // bytes (4 for float/uint32)
  char type;   // 'F' for float, 'U' for unsigned
  int offset;  // byte offset within point record
};

/**
 * @brief Build field descriptors based on cloud channels
 */
inline std::vector<FieldDesc> buildFieldDescriptors(const PointCloud& cloud) {
  std::vector<FieldDesc> fields;
  int offset = 0;

  // XYZ (always present)
  fields.push_back({"x", 4, 'F', offset});
  offset += 4;
  fields.push_back({"y", 4, 'F', offset});
  offset += 4;
  fields.push_back({"z", 4, 'F', offset});
  offset += 4;

  // Intensity
  if (cloud.hasIntensity()) {
    fields.push_back({"intensity", 4, 'F', offset});
    offset += 4;
  }

  // RGB (packed float)
  if (cloud.hasColor()) {
    fields.push_back({"rgb", 4, 'F', offset});
    offset += 4;
  }

  // Normal
  if (cloud.hasNormal()) {
    fields.push_back({"normal_x", 4, 'F', offset});
    offset += 4;
    fields.push_back({"normal_y", 4, 'F', offset});
    offset += 4;
    fields.push_back({"normal_z", 4, 'F', offset});
    offset += 4;
  }

  // Label (uint32)
  if (cloud.hasLabel()) {
    fields.push_back({"label", 4, 'U', offset});
    offset += 4;
  }

  return fields;
}

/**
 * @brief Calculate total bytes per point from field descriptors
 */
inline size_t bytesPerPoint(const std::vector<FieldDesc>& fields) {
  if (fields.empty()) return 0;
  const auto& last = fields.back();
  return last.offset + last.size;
}

/**
 * @brief Write PCD header based on field descriptors
 */
inline void writePCDHeader(std::ostream& os,
                           const std::vector<FieldDesc>& fields,
                           size_t num_points) {
  os << "VERSION 0.7\n";

  // FIELDS line
  os << "FIELDS";
  for (const auto& f : fields) {
    os << " " << f.name;
  }
  os << "\n";

  // SIZE line
  os << "SIZE";
  for (const auto& f : fields) {
    os << " " << f.size;
  }
  os << "\n";

  // TYPE line
  os << "TYPE";
  for (const auto& f : fields) {
    os << " " << f.type;
  }
  os << "\n";

  // COUNT line
  os << "COUNT";
  for (size_t i = 0; i < fields.size(); ++i) {
    os << " 1";
  }
  os << "\n";

  os << "WIDTH " << num_points << "\n";
  os << "HEIGHT 1\n";
  os << "VIEWPOINT 0 0 0 1 0 0 0\n";
  os << "POINTS " << num_points << "\n";
  os << "DATA binary\n";
}

}  // namespace detail

// =============================================================================
// Load PCD Implementation
// =============================================================================

inline PointCloud loadPCD(const std::string& filename) {
  std::ifstream fs(filename, std::ios::binary);
  if (!fs.is_open()) {
    throw IOError("Cannot open file: " + filename);
  }

  std::string line;
  size_t width = 0, height = 0, points = 0;
  bool binary = false;

  // Field indices (-1 means not found)
  int x_idx = -1, y_idx = -1, z_idx = -1;
  int intensity_idx = -1, rgb_idx = -1;
  int normal_x_idx = -1, normal_y_idx = -1, normal_z_idx = -1;
  int label_idx = -1;

  std::vector<std::string> fields;
  std::vector<int> sizes;
  std::vector<char> types;

  // Parse Header
  while (std::getline(fs, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string tag;
    ss >> tag;

    if (tag == "FIELDS") {
      std::string field;
      while (ss >> field) {
        // Convert to lowercase for case-insensitive matching
        std::transform(field.begin(), field.end(), field.begin(), ::tolower);
        fields.push_back(field);
      }
      // Find field indices
      for (size_t i = 0; i < fields.size(); ++i) {
        if (fields[i] == "x")
          x_idx = static_cast<int>(i);
        else if (fields[i] == "y")
          y_idx = static_cast<int>(i);
        else if (fields[i] == "z")
          z_idx = static_cast<int>(i);
        else if (fields[i] == "intensity" || fields[i] == "i")
          intensity_idx = static_cast<int>(i);
        else if (fields[i] == "rgb")
          rgb_idx = static_cast<int>(i);
        else if (fields[i] == "normal_x" || fields[i] == "nx")
          normal_x_idx = static_cast<int>(i);
        else if (fields[i] == "normal_y" || fields[i] == "ny")
          normal_y_idx = static_cast<int>(i);
        else if (fields[i] == "normal_z" || fields[i] == "nz")
          normal_z_idx = static_cast<int>(i);
        else if (fields[i] == "label")
          label_idx = static_cast<int>(i);
      }
    } else if (tag == "SIZE") {
      int sz;
      while (ss >> sz) {
        sizes.push_back(sz);
      }
    } else if (tag == "TYPE") {
      char t;
      while (ss >> t) {
        types.push_back(t);
      }
    } else if (tag == "WIDTH")
      ss >> width;
    else if (tag == "HEIGHT")
      ss >> height;
    else if (tag == "POINTS")
      ss >> points;
    else if (tag == "DATA") {
      std::string type;
      ss >> type;
      if (type == "binary") binary = true;
      break;  // End of header
    }
  }

  if (points == 0) {
    if (width > 0 && height > 0)
      points = width * height;
    else {
      throw IOError("Could not determine point count in: " + filename);
    }
  }

  if (!binary) {
    throw IOError("Only binary PCD format is supported: " + filename);
  }

  // Calculate bytes per point from SIZE field
  size_t bytes_per_point = 0;
  for (int sz : sizes) {
    bytes_per_point += sz;
  }

  if (bytes_per_point == 0) {
    // Fallback: infer from file size
    std::streampos data_start = fs.tellg();
    fs.seekg(0, std::ios::end);
    std::streampos data_end = fs.tellg();
    size_t data_size = data_end - data_start;
    fs.seekg(data_start);
    bytes_per_point = data_size / points;
  }

  // Prepare cloud
  PointCloud cloud;
  cloud.reserve(points);
  cloud.resize(points);

  const bool has_intensity = (intensity_idx >= 0);
  const bool has_rgb = (rgb_idx >= 0);
  const bool has_normal =
      (normal_x_idx >= 0 && normal_y_idx >= 0 && normal_z_idx >= 0);
  const bool has_label = (label_idx >= 0);

  if (has_intensity) cloud.enableIntensity();
  if (has_rgb) cloud.enableColor();
  if (has_normal) cloud.enableNormal();
  if (has_label) cloud.enableLabel();

  // Determine field reading strategy
  bool use_field_parsing = (x_idx >= 0 && y_idx >= 0 && z_idx >= 0);

  if (bytes_per_point == 12 && !has_intensity && !has_rgb && !has_normal &&
      !has_label && !use_field_parsing) {
    // Fast path: Packed XYZ (12 bytes) - direct read into cloud
    fs.read(reinterpret_cast<char*>(cloud.data()), points * sizeof(Point));
  } else {
    // General path: Chunked reading
    constexpr size_t CHUNK_POINTS = 10000;
    std::vector<uint8_t> buffer(CHUNK_POINTS * bytes_per_point);

    // Calculate byte offsets for each field
    std::vector<size_t> offsets(fields.size(), 0);
    size_t current_offset = 0;
    for (size_t i = 0; i < fields.size() && i < sizes.size(); ++i) {
      offsets[i] = current_offset;
      current_offset += sizes[i];
    }

    // Field indices with byte offsets
    const int xi = (x_idx >= 0) ? x_idx : 0;
    const int yi = (y_idx >= 0) ? y_idx : 1;
    const int zi = (z_idx >= 0) ? z_idx : 2;

    size_t processed = 0;
    while (processed < points) {
      const size_t batch = std::min(CHUNK_POINTS, points - processed);
      const size_t batch_bytes = batch * bytes_per_point;

      fs.read(reinterpret_cast<char*>(buffer.data()), batch_bytes);

      // Parse chunk into cloud
      for (size_t i = 0; i < batch; ++i) {
        const uint8_t* point_data = buffer.data() + i * bytes_per_point;
        const size_t cloud_idx = processed + i;

        // XYZ
        std::memcpy(&cloud[cloud_idx].x(), point_data + offsets[xi],
                    sizeof(float));
        std::memcpy(&cloud[cloud_idx].y(), point_data + offsets[yi],
                    sizeof(float));
        std::memcpy(&cloud[cloud_idx].z(), point_data + offsets[zi],
                    sizeof(float));

        // Intensity
        if (has_intensity) {
          std::memcpy(&cloud.intensity()[cloud_idx],
                      point_data + offsets[intensity_idx], sizeof(float));
        }

        // RGB
        if (has_rgb) {
          float rgb_packed;
          std::memcpy(&rgb_packed, point_data + offsets[rgb_idx],
                      sizeof(float));
          uint8_t r, g, b;
          detail::unpackFloatToRGB(rgb_packed, r, g, b);
          cloud.color()[cloud_idx] = Color(r, g, b);
        }

        // Normal
        if (has_normal) {
          Eigen::Vector3f normal;
          std::memcpy(&normal.x(), point_data + offsets[normal_x_idx],
                      sizeof(float));
          std::memcpy(&normal.y(), point_data + offsets[normal_y_idx],
                      sizeof(float));
          std::memcpy(&normal.z(), point_data + offsets[normal_z_idx],
                      sizeof(float));
          cloud.normal()[cloud_idx] = normal;
        }

        // Label
        if (has_label) {
          uint32_t label_val;
          std::memcpy(&label_val, point_data + offsets[label_idx],
                      sizeof(uint32_t));
          cloud.label()[cloud_idx] = Label(label_val);
        }
      }

      processed += batch;
    }
  }

  return cloud;
}

// =============================================================================
// Save PCD Implementation
// =============================================================================

inline void savePCD(const std::string& filename, const PointCloud& cloud) {
  std::ofstream fs(filename, std::ios::binary);
  if (!fs.is_open()) {
    throw IOError("Cannot open file for writing: " + filename);
  }

  // Build field descriptors dynamically
  auto fields = detail::buildFieldDescriptors(cloud);
  const size_t point_size = detail::bytesPerPoint(fields);

  // Write header
  detail::writePCDHeader(fs, fields, cloud.size());

  // Write body - chunked for memory efficiency
  constexpr size_t CHUNK_SIZE = 10000;
  std::vector<uint8_t> buffer(CHUNK_SIZE * point_size);

  const bool has_intensity = cloud.hasIntensity();
  const bool has_color = cloud.hasColor();
  const bool has_normal = cloud.hasNormal();
  const bool has_label = cloud.hasLabel();

  size_t points_written = 0;
  while (points_written < cloud.size()) {
    const size_t batch = std::min(CHUNK_SIZE, cloud.size() - points_written);

    // Pack points into buffer
    for (size_t i = 0; i < batch; ++i) {
      uint8_t* point_data = buffer.data() + i * point_size;
      const size_t cloud_idx = points_written + i;

      // XYZ
      std::memcpy(point_data, &cloud[cloud_idx].x(), sizeof(float));
      std::memcpy(point_data + 4, &cloud[cloud_idx].y(), sizeof(float));
      std::memcpy(point_data + 8, &cloud[cloud_idx].z(), sizeof(float));

      size_t offset = 12;

      // Intensity
      if (has_intensity) {
        std::memcpy(point_data + offset, &cloud.intensity()[cloud_idx],
                    sizeof(float));
        offset += 4;
      }

      // RGB
      if (has_color) {
        const auto& c = cloud.color()[cloud_idx];
        float rgb_packed = detail::packRGBToFloat(c.r(), c.g(), c.b());
        std::memcpy(point_data + offset, &rgb_packed, sizeof(float));
        offset += 4;
      }

      // Normal
      if (has_normal) {
        const auto& n = cloud.normal()[cloud_idx];
        std::memcpy(point_data + offset, &n.x(), sizeof(float));
        std::memcpy(point_data + offset + 4, &n.y(), sizeof(float));
        std::memcpy(point_data + offset + 8, &n.z(), sizeof(float));
        offset += 12;
      }

      // Label
      if (has_label) {
        uint32_t label_val = cloud.label()[cloud_idx].val;
        std::memcpy(point_data + offset, &label_val, sizeof(uint32_t));
        offset += 4;
      }
    }

    // Write chunk
    fs.write(reinterpret_cast<const char*>(buffer.data()), batch * point_size);
    points_written += batch;
  }
}

}  // namespace io
}  // namespace npcl

#endif  // NANOPCL_IO_IMPL_PCD_IO_IMPL_HPP
