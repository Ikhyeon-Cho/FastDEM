// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Implementation details for BIN I/O.
// Do not include this file directly; include <nanopcl/io/bin_io.hpp>

#ifndef NANOPCL_IO_IMPL_BIN_IO_IMPL_HPP
#define NANOPCL_IO_IMPL_BIN_IO_IMPL_HPP

#include <fstream>
#include <vector>

namespace npcl {
namespace io {

inline PointCloud loadBIN(const std::string& filename, bool load_intensity) {
  std::ifstream fs(filename, std::ios::binary | std::ios::ate);
  if (!fs.is_open()) {
    throw IOError("Cannot open file: " + filename);
  }

  std::streampos size = fs.tellg();
  fs.seekg(0, std::ios::beg);

  // Each point is 4 floats (x, y, z, i) = 16 bytes
  size_t points_num = size / (4 * sizeof(float));

  std::vector<float> buffer(points_num * 4);
  fs.read(reinterpret_cast<char*>(buffer.data()), size);

  PointCloud cloud;
  cloud.reserve(points_num);
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

  return cloud;
}

inline void saveBIN(const std::string& filename, const PointCloud& cloud) {
  std::ofstream fs(filename, std::ios::binary);
  if (!fs.is_open()) {
    throw IOError("Cannot open file for writing: " + filename);
  }

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

  fs.write(reinterpret_cast<const char*>(buffer.data()),
           buffer.size() * sizeof(float));
}

}  // namespace io
}  // namespace npcl

#endif  // NANOPCL_IO_IMPL_BIN_IO_IMPL_HPP
