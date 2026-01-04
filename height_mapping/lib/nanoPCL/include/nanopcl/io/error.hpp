// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_IO_ERROR_HPP
#define NANOPCL_IO_ERROR_HPP

#include <string>

namespace nanopcl {
namespace io {

namespace detail {
/// Thread-local error message storage
inline thread_local std::string g_last_error;
}  // namespace detail

/**
 * @brief Get the last I/O error message.
 *
 * Returns the error message from the most recent failed I/O operation
 * in the current thread. Empty string if no error occurred.
 *
 * @return Reference to the last error message
 */
inline const std::string& lastError() noexcept { return detail::g_last_error; }

/**
 * @brief Clear the last error message.
 */
inline void clearError() noexcept { detail::g_last_error.clear(); }

namespace detail {
/// Internal: Set error message (used by I/O functions)
inline void setLastError(const std::string& msg) { g_last_error = msg; }
}  // namespace detail

}  // namespace io
}  // namespace nanopcl

#endif  // NANOPCL_IO_ERROR_HPP
