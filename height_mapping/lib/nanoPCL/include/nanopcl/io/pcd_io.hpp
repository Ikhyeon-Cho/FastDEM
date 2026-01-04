// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_IO_PCD_IO_HPP
#define NANOPCL_IO_PCD_IO_HPP

#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/io/error.hpp"

namespace nanopcl {
namespace io {

/**
 * @brief Load PointCloud from binary PCD file.
 *
 * Supports PCD v0.7 binary format with automatic intensity detection.
 * Parses FIELDS header to detect x, y, z, and optionally intensity fields.
 *
 * @param filename Path to PCD file
 * @param cloud [out] Output point cloud (intensity enabled if detected)
 * @return true if successful
 */
inline bool loadPCD(const std::string& filename, PointCloud& cloud) {
  std::ifstream fs(filename, std::ios::binary);
  if (!fs.is_open()) {
    detail::setLastError("Cannot open file: " + filename);
    return false;
  }

  std::string line;
  size_t width = 0, height = 0, points = 0;
  bool binary = false;

  // Field indices (-1 means not found)
  int x_idx = -1, y_idx = -1, z_idx = -1, intensity_idx = -1;
  std::vector<std::string> fields;
  std::vector<int> sizes;

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
      }
    } else if (tag == "SIZE") {
      int sz;
      while (ss >> sz) {
        sizes.push_back(sz);
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
      detail::setLastError("Could not determine point count in: " + filename);
      return false;
    }
  }

  if (!binary) {
    detail::setLastError("Only binary PCD format is supported: " + filename);
    return false;
  }

  // Calculate bytes per point from SIZE field
  size_t bytes_per_point_from_header = 0;
  for (int sz : sizes) {
    bytes_per_point_from_header += sz;
  }

  // Check remaining file size
  std::streampos data_start = fs.tellg();
  fs.seekg(0, std::ios::end);
  std::streampos data_end = fs.tellg();
  size_t data_size = data_end - data_start;
  fs.seekg(data_start);

  size_t bytes_per_point = data_size / points;

  // Use header info if available, otherwise use file size inference
  if (bytes_per_point_from_header > 0) {
    bytes_per_point = bytes_per_point_from_header;
  }

  // Prepare cloud
  cloud.clear();
  cloud.reserve(points);
  cloud.resize(points);

  bool has_intensity = (intensity_idx >= 0);
  if (has_intensity) {
    cloud.enableIntensity();
  }

  // Determine reading strategy based on field indices
  bool use_field_parsing = (x_idx >= 0 && y_idx >= 0 && z_idx >= 0);

  if (bytes_per_point == 12 && !has_intensity && !use_field_parsing) {
    // Fast path: Packed XYZ (12 bytes)
    fs.read(reinterpret_cast<char*>(cloud.data()), points * sizeof(Point));
  } else {
    // General path: Read whole buffer and parse by field indices
    std::vector<float> buffer(points * (bytes_per_point / sizeof(float)));
    fs.read(reinterpret_cast<char*>(buffer.data()), data_size);

    size_t floats_per_point = bytes_per_point / sizeof(float);

    // Use field indices if available, otherwise assume x=0, y=1, z=2, i=3
    int xi = (x_idx >= 0) ? x_idx : 0;
    int yi = (y_idx >= 0) ? y_idx : 1;
    int zi = (z_idx >= 0) ? z_idx : 2;
    int ii = (intensity_idx >= 0) ? intensity_idx : 3;

    for (size_t i = 0; i < points; ++i) {
      size_t base = i * floats_per_point;
      cloud[i].x() = buffer[base + xi];
      cloud[i].y() = buffer[base + yi];
      cloud[i].z() = buffer[base + zi];

      if (has_intensity && static_cast<size_t>(ii) < floats_per_point) {
        cloud.intensity()[i] = buffer[base + ii];
      }
    }
  }

  return true;
}

/**
 * @brief Save PointCloud to binary PCD file.
 *
 * Saves as uncompressed binary. Automatically includes intensity field
 * if the cloud has intensity enabled.
 *
 * @param filename Path to output PCD file
 * @param cloud Point cloud to save
 * @return true if successful
 */
inline bool savePCD(const std::string& filename, const PointCloud& cloud) {
  std::ofstream fs(filename, std::ios::binary);
  if (!fs.is_open()) return false;

  bool has_intensity = cloud.hasIntensity();

  // Header
  fs << "VERSION 0.7\n";
  if (has_intensity) {
    fs << "FIELDS x y z intensity\n";
    fs << "SIZE 4 4 4 4\n";
    fs << "TYPE F F F F\n";
    fs << "COUNT 1 1 1 1\n";
  } else {
    fs << "FIELDS x y z\n";
    fs << "SIZE 4 4 4\n";
    fs << "TYPE F F F\n";
    fs << "COUNT 1 1 1\n";
  }
  fs << "WIDTH " << cloud.size() << "\n";
  fs << "HEIGHT 1\n";
  fs << "VIEWPOINT 0 0 0 1 0 0 0\n";
  fs << "POINTS " << cloud.size() << "\n";
  fs << "DATA binary\n";

  // Body
  if (has_intensity) {
    // Write XYZI interleaved
    const auto& intensities = cloud.intensity();
    for (size_t i = 0; i < cloud.size(); ++i) {
      fs.write(reinterpret_cast<const char*>(&cloud[i]), sizeof(Point));
      fs.write(reinterpret_cast<const char*>(&intensities[i]), sizeof(float));
    }
  } else {
    // Fast path: Packed XYZ
    fs.write(reinterpret_cast<const char*>(cloud.data()),
             cloud.size() * sizeof(Point));
  }

  return true;
}

}  // namespace io
}  // namespace nanopcl

#endif  // NANOPCL_IO_PCD_IO_HPP
