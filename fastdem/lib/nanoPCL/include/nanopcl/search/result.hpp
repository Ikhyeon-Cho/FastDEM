// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_SEARCH_RESULT_HPP
#define NANOPCL_SEARCH_RESULT_HPP

#include <cstdint>

namespace nanopcl {
namespace search {

struct NearestResult {
  uint32_t index;
  float dist_sq;
};

} // namespace search
} // namespace nanopcl

#endif // NANOPCL_SEARCH_RESULT_HPP
