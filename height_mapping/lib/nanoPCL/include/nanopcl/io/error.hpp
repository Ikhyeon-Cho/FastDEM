// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_IO_ERROR_HPP
#define NANOPCL_IO_ERROR_HPP

#include <stdexcept>
#include <string>

namespace npcl {
namespace io {

/**
 * @brief Exception thrown for I/O errors
 *
 * Used by loadPCD, savePCD, loadBIN, saveBIN, etc.
 */
class IOError : public std::runtime_error {
 public:
  explicit IOError(const std::string& msg) : std::runtime_error(msg) {}
  explicit IOError(const char* msg) : std::runtime_error(msg) {}
};

}  // namespace io
}  // namespace npcl

#endif  // NANOPCL_IO_ERROR_HPP
