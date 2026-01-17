// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_SEARCH_RESULT_HPP
#define NANOPCL_SEARCH_RESULT_HPP

#include <cstdint>

namespace npcl {
namespace search {

/**
 * @brief Result of nearest neighbor search
 *
 * Contains the index of the nearest point and the squared distance.
 * Using squared distance avoids unnecessary sqrt() computation.
 *
 * No default constructor - always returned via std::optional.
 */
struct NearestResult {
  uint32_t index;  ///< Index of the nearest point in the cloud
  float dist_sq;   ///< Squared distance to the point
};

}  // namespace search
}  // namespace npcl

#endif  // NANOPCL_SEARCH_RESULT_HPP
