// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// VoxelKey: A value type representing a discrete voxel coordinate.
//
// This is a core type used by spatial data structures (VoxelHash, VoxelMap,
// etc.) to ensure consistent voxelization across the library.
//
// Example usage:
//   float resolution = 0.1f;
//   float inv_resolution = 1.0f / resolution;
//
//   VoxelKey key = VoxelKey::from(point, inv_resolution);
//   uint64_t hash = key.hash();

#ifndef NANOPCL_CORE_VOXEL_KEY_HPP
#define NANOPCL_CORE_VOXEL_KEY_HPP

#include <Eigen/Core>
#include <cmath>
#include <cstdint>

#include "nanopcl/core/point.hpp"

namespace npcl {

/**
 * @brief Discrete voxel coordinate with hash support
 *
 * Represents a 3D integer coordinate in voxel space. Provides consistent
 * point-to-voxel mapping and spatial hashing across the library.
 */
struct VoxelKey {
  Eigen::Vector3i coord;

  /// Spatial hash primes (Teschner et al., "Optimized Spatial Hashing")
  static constexpr uint64_t HASH_PRIME_X = 73856093;
  static constexpr uint64_t HASH_PRIME_Y = 19349663;
  static constexpr uint64_t HASH_PRIME_Z = 83492791;

  /**
   * @brief Create VoxelKey from a 3D point
   *
   * Maps a continuous 3D point to a discrete voxel coordinate.
   * Formula: floor(coord * inv_resolution)
   *
   * @param p Point in continuous space
   * @param inv_resolution Inverse of voxel resolution (1.0 / resolution)
   * @return VoxelKey representing the voxel containing the point
   */
  [[nodiscard]] static VoxelKey from(const Point& p, float inv_resolution) {
    return {
        Eigen::Vector3i(static_cast<int>(std::floor(p.x() * inv_resolution)),
                        static_cast<int>(std::floor(p.y() * inv_resolution)),
                        static_cast<int>(std::floor(p.z() * inv_resolution)))};
  }

  /**
   * @brief Compute spatial hash value
   *
   * Implements "Optimized Spatial Hashing" (Teschner et al., 2003).
   * Designed for fast collision detection and neighbor search in 3D space.
   *
   * @return 64-bit hash suitable for hash tables
   */
  [[nodiscard]] uint64_t hash() const {
    return (static_cast<uint64_t>(coord[0]) * HASH_PRIME_X) ^
           (static_cast<uint64_t>(coord[1]) * HASH_PRIME_Y) ^
           (static_cast<uint64_t>(coord[2]) * HASH_PRIME_Z);
  }

  /// Equality comparison
  bool operator==(const VoxelKey& other) const { return coord == other.coord; }
  bool operator!=(const VoxelKey& other) const { return coord != other.coord; }

  /// Invalid key marker (for hash table sentinel)
  [[nodiscard]] static VoxelKey invalid() {
    return {Eigen::Vector3i::Constant(std::numeric_limits<int>::min())};
  }

  /// Check if this key is the invalid sentinel
  [[nodiscard]] bool isValid() const {
    return coord[0] != std::numeric_limits<int>::min();
  }
};

}  // namespace npcl

#endif  // NANOPCL_CORE_VOXEL_KEY_HPP
