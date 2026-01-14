// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_REGISTRATION_CORRESPONDENCE_HPP
#define NANOPCL_REGISTRATION_CORRESPONDENCE_HPP

#include <cstdint>
#include <limits>
#include <vector>

namespace npcl {
namespace registration {

/**
 * @brief Point correspondence between source and target clouds
 *
 * Stores indices rather than point coordinates to allow access to
 * additional attributes (normals, covariances) from the original clouds.
 */
struct Correspondence {
  uint32_t source_idx;  ///< Index in source cloud
  uint32_t target_idx;  ///< Index in target cloud
  float dist_sq;        ///< Squared distance between matched points
};

/// Alias for correspondence container
using Correspondences = std::vector<Correspondence>;

/// Invalid correspondence marker (used for parallel compaction)
constexpr uint32_t INVALID_INDEX = std::numeric_limits<uint32_t>::max();

}  // namespace registration
}  // namespace npcl

#endif  // NANOPCL_REGISTRATION_CORRESPONDENCE_HPP
