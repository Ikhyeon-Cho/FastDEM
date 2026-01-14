// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_IO_PCD_IO_HPP
#define NANOPCL_IO_PCD_IO_HPP

#include <string>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/io/error.hpp"

namespace npcl {
namespace io {

/**
 * @brief Load PointCloud from a binary PCD file
 *
 * Automatically detects and enables channels based on the PCD header.
 * Supports: XYZ, Intensity, Ring, Time, Color (RGB), Label, Normals.
 *
 * @param filename Path to the .pcd file (binary or binary_compressed preferred)
 * @return Loaded point cloud
 *
 * @throws IOError if the file is missing, header is malformed, or data size
 * mismatches.
 */
[[nodiscard]] PointCloud loadPCD(const std::string& filename);

/**
 * @brief Save PointCloud to a binary PCD file
 *
 * Saves the point cloud using the BINARY format. Only active channels
 * are written to the file to save space.
 *
 * @param filename Output path for the .pcd file
 * @param cloud Point cloud data to save
 *
 * @throws IOError if the directory is not writable or disk is full.
 */
void savePCD(const std::string& filename, const PointCloud& cloud);

}  // namespace io
}  // namespace npcl

#include "nanopcl/io/impl/pcd_io_impl.hpp"

#endif  // NANOPCL_IO_PCD_IO_HPP
