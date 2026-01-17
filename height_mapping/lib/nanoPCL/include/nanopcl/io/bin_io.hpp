// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_IO_BIN_IO_HPP
#define NANOPCL_IO_BIN_IO_HPP

#include <string>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/io/error.hpp"

namespace npcl {
namespace io {

/**
 * @brief Load PointCloud from KITTI binary file (.bin)
 *
 * @param filename Path to BIN file
 * @param load_intensity Enable intensity channel loading (default: true)
 * @return Loaded point cloud
 * @throws IOError if file cannot be opened
 */
[[nodiscard]] PointCloud loadBIN(const std::string& filename,
                                 bool load_intensity = true);

/**
 * @brief Save PointCloud to KITTI binary file (.bin)
 *
 * @param filename Path to output BIN file
 * @param cloud Point cloud to save
 * @throws IOError if file cannot be opened for writing
 */
void saveBIN(const std::string& filename, const PointCloud& cloud);

}  // namespace io
}  // namespace npcl

#include "nanopcl/io/impl/bin_io_impl.hpp"

#endif  // NANOPCL_IO_BIN_IO_HPP
