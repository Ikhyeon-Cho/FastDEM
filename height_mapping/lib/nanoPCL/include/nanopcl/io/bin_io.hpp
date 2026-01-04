// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_IO_BIN_IO_HPP
#define NANOPCL_IO_BIN_IO_HPP

#include <fstream>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/io/error.hpp"

namespace nanopcl {
namespace io {

/**
 * @brief Load PointCloud from KITTI binary file (.bin).
 *
 * Format: N * [x, y, z, intensity] (floats)
 *
 * @param filename Path to BIN file
 * @param cloud [out] Output point cloud
 * @param load_intensity Enable intensity channel loading (default: true)
 * @return true if successful
 */
inline bool loadBIN(const std::string& filename, PointCloud& cloud,
                    bool load_intensity = true) {
  std::ifstream fs(filename, std::ios::binary | std::ios::ate);
  if (!fs.is_open()) {
    detail::setLastError("Cannot open file: " + filename);
    return false;
  }

  std::streampos size = fs.tellg();
  fs.seekg(0, std::ios::beg);

  // Each point is 4 floats (x, y, z, i) = 16 bytes
  size_t points_num = size / (4 * sizeof(float));

  cloud.clear();
  cloud.reserve(points_num);

  std::vector<float> buffer(points_num * 4);
  fs.read(reinterpret_cast<char*>(buffer.data()), size);

  cloud.resize(points_num);
  if (load_intensity) {
    cloud.enableIntensity();
  }

  for (size_t i = 0; i < points_num; ++i) {
    cloud[i].x() = buffer[i * 4 + 0];
    cloud[i].y() = buffer[i * 4 + 1];
    cloud[i].z() = buffer[i * 4 + 2];

    if (load_intensity) {
      cloud.intensity()[i] = buffer[i * 4 + 3];
    }
  }

  return true;
}

/**
 * @brief Save PointCloud to KITTI binary file (.bin).
 * 
 * Format: x, y, z, intensity (0.0 if not available)
 */
inline bool saveBIN(const std::string& filename, const PointCloud& cloud) {
  std::ofstream fs(filename, std::ios::binary);
  if (!fs.is_open()) return false;

  const bool has_intensity = cloud.hasIntensity();
  std::vector<float> buffer;
  buffer.reserve(cloud.size() * 4);

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& pt = cloud[i];
    buffer.push_back(pt.x());
    buffer.push_back(pt.y());
    buffer.push_back(pt.z());
    buffer.push_back(has_intensity ? cloud.intensity()[i] : 0.0f);
  }

  fs.write(reinterpret_cast<const char*>(buffer.data()), buffer.size() * sizeof(float));
  return true;
}

}  // namespace io
}  // namespace nanopcl

#endif  // NANOPCL_IO_BIN_IO_HPP
