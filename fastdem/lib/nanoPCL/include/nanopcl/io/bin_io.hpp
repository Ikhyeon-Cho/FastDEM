// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// BIN file I/O for LiDAR dataset formats.
//
// Supported formats:
//   - KITTI: 4 floats per point (x, y, z, intensity)
//   - NuScenes: 5 floats per point (x, y, z, intensity, ring)
//   - Custom: configurable via BINLoadOptions
//
// Example:
//   auto cloud = io::loadKITTI("000000.bin");
//   auto cloud = io::loadNuScenes("sweep.bin");
//   auto cloud = io::loadBIN("custom.bin", {.fields_per_point = 6});

#ifndef NANOPCL_IO_BIN_IO_HPP
#define NANOPCL_IO_BIN_IO_HPP

#include <cstring>
#include <fstream>
#include <string>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/io/pcd_io.hpp"  // For IOException

namespace nanopcl {
namespace io {

// ========
// Options
// ========

/// @brief Options for loading BIN files with different layouts
struct BINLoadOptions {
  int fields_per_point = 4;    ///< Number of float32 fields per point
  bool load_intensity = true;  ///< Load intensity from file
  int intensity_index = 3;     ///< 0-based index of intensity field
  bool load_ring = false;      ///< Load ring/channel from file
  int ring_index = 4;          ///< 0-based index of ring field
};

// ===============
// Load Functions
// ===============

/// @brief Load BIN from stream with options
/// @param is Input stream
/// @param options Load options
/// @return Loaded point cloud
/// @throws IOException on read error
inline PointCloud loadBIN(std::istream& is, const BINLoadOptions& options = {}) {
  if (!is) {
    throw IOException("Invalid input stream");
  }

  // Get file size
  is.seekg(0, std::ios::end);
  std::streamsize file_size = is.tellg();
  is.seekg(0, std::ios::beg);

  if (file_size <= 0) {
    return PointCloud();
  }

  const size_t point_size = options.fields_per_point * sizeof(float);
  const size_t num_points = static_cast<size_t>(file_size) / point_size;

  if (num_points == 0) {
    return PointCloud();
  }

  // Read all data at once
  std::vector<float> buffer(num_points * options.fields_per_point);
  if (!is.read(reinterpret_cast<char*>(buffer.data()), file_size)) {
    throw IOException("Failed to read BIN data");
  }

  // Build point cloud
  PointCloud cloud;
  if (options.load_intensity) cloud.useIntensity();
  if (options.load_ring) cloud.useRing();
  cloud.reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    const size_t offset = i * options.fields_per_point;
    float x = buffer[offset];
    float y = buffer[offset + 1];
    float z = buffer[offset + 2];

    cloud.add(x, y, z);

    if (options.load_intensity && options.intensity_index < options.fields_per_point) {
      cloud.intensities().back() = buffer[offset + options.intensity_index];
    }
    if (options.load_ring && options.ring_index < options.fields_per_point) {
      cloud.rings().back() = static_cast<uint16_t>(buffer[offset + options.ring_index]);
    }
  }

  return cloud;
}

/// @brief Load BIN from file with options
/// @param path File path
/// @param options Load options
/// @return Loaded point cloud
/// @throws IOException on file/read error
inline PointCloud loadBIN(const std::string& path,
                          const BINLoadOptions& options = {}) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    throw IOException("Cannot open file: " + path);
  }
  return loadBIN(ifs, options);
}

// =========================
// Dataset-specific Helpers
// =========================

/// @brief Load KITTI format BIN file (x, y, z, intensity)
/// @param path File path
/// @return Point cloud with intensity
inline PointCloud loadKITTI(const std::string& path) {
  return loadBIN(path, {
      .fields_per_point = 4,
      .load_intensity = true,
      .intensity_index = 3,
      .load_ring = false,
  });
}

/// @brief Load NuScenes format BIN file (x, y, z, intensity, ring)
/// @param path File path
/// @return Point cloud with intensity and ring
inline PointCloud loadNuScenes(const std::string& path) {
  return loadBIN(path, {
      .fields_per_point = 5,
      .load_intensity = true,
      .intensity_index = 3,
      .load_ring = true,
      .ring_index = 4,
  });
}

// ===============
// Save Functions
// ===============

/// @brief Save to BIN stream (KITTI format: x, y, z, intensity)
/// @param os Output stream
/// @param cloud Point cloud to save
/// @throws IOException on write error
inline void saveBIN(std::ostream& os, const PointCloud& cloud) {
  if (!os) {
    throw IOException("Invalid output stream");
  }

  const size_t n = cloud.size();
  const bool has_intensity = cloud.hasIntensity();

  for (size_t i = 0; i < n; ++i) {
    const auto& p = cloud.point(i);
    float x = p.x();
    float y = p.y();
    float z = p.z();
    float intensity = has_intensity ? cloud.intensity(i) : 0.0f;

    os.write(reinterpret_cast<const char*>(&x), sizeof(float));
    os.write(reinterpret_cast<const char*>(&y), sizeof(float));
    os.write(reinterpret_cast<const char*>(&z), sizeof(float));
    os.write(reinterpret_cast<const char*>(&intensity), sizeof(float));
  }

  if (!os) {
    throw IOException("Error writing BIN data");
  }
}

/// @brief Save to BIN file (KITTI format)
/// @param path File path
/// @param cloud Point cloud to save
/// @throws IOException on file/write error
inline void saveBIN(const std::string& path, const PointCloud& cloud) {
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) {
    throw IOException("Cannot create file: " + path);
  }
  saveBIN(ofs, cloud);
}

/// @brief Save to BIN stream with ring (NuScenes format: x, y, z, intensity, ring)
/// @param os Output stream
/// @param cloud Point cloud to save
/// @throws IOException on write error
inline void saveBINWithRing(std::ostream& os, const PointCloud& cloud) {
  if (!os) {
    throw IOException("Invalid output stream");
  }

  const size_t n = cloud.size();
  const bool has_intensity = cloud.hasIntensity();
  const bool has_ring = cloud.hasRing();

  for (size_t i = 0; i < n; ++i) {
    const auto& p = cloud.point(i);
    float x = p.x();
    float y = p.y();
    float z = p.z();
    float intensity = has_intensity ? cloud.intensity(i) : 0.0f;
    float ring = has_ring ? static_cast<float>(cloud.ring(i)) : 0.0f;

    os.write(reinterpret_cast<const char*>(&x), sizeof(float));
    os.write(reinterpret_cast<const char*>(&y), sizeof(float));
    os.write(reinterpret_cast<const char*>(&z), sizeof(float));
    os.write(reinterpret_cast<const char*>(&intensity), sizeof(float));
    os.write(reinterpret_cast<const char*>(&ring), sizeof(float));
  }

  if (!os) {
    throw IOException("Error writing BIN data");
  }
}

/// @brief Save to BIN file with ring (NuScenes format)
/// @param path File path
/// @param cloud Point cloud to save
/// @throws IOException on file/write error
inline void saveBINWithRing(const std::string& path, const PointCloud& cloud) {
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) {
    throw IOException("Cannot create file: " + path);
  }
  saveBINWithRing(ofs, cloud);
}

}  // namespace io
}  // namespace nanopcl

#endif  // NANOPCL_IO_BIN_IO_HPP
