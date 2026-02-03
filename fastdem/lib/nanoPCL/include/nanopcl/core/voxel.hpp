// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_VOXEL_HPP
#define NANOPCL_CORE_VOXEL_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "nanopcl/core/types.hpp"

namespace nanopcl {
namespace voxel {

// Voxel size limits
constexpr float MIN_SIZE = 0.001f; // 1mm minimum
constexpr float MAX_SIZE = 100.0f; // 100m maximum

// Coordinate packing constants
constexpr int32_t COORD_OFFSET = 1 << 20;
constexpr int32_t COORD_MIN = -COORD_OFFSET;
constexpr int32_t COORD_MAX = COORD_OFFSET - 1;
constexpr uint64_t COORD_MASK = 0x1FFFFF;
constexpr uint64_t INVALID_KEY = ~uint64_t(0);

// Pack coordinates into 64-bit key: [z:21][y:21][x:21]
inline uint64_t pack(float x, float y, float z, float inv_voxel_size) {
  int32_t ix = static_cast<int32_t>(std::floor(x * inv_voxel_size));
  int32_t iy = static_cast<int32_t>(std::floor(y * inv_voxel_size));
  int32_t iz = static_cast<int32_t>(std::floor(z * inv_voxel_size));

  ix = std::clamp(ix, COORD_MIN, COORD_MAX);
  iy = std::clamp(iy, COORD_MIN, COORD_MAX);
  iz = std::clamp(iz, COORD_MIN, COORD_MAX);

  uint64_t ux = static_cast<uint64_t>(ix + COORD_OFFSET);
  uint64_t uy = static_cast<uint64_t>(iy + COORD_OFFSET);
  uint64_t uz = static_cast<uint64_t>(iz + COORD_OFFSET);

  return (uz << 42) | (uy << 21) | ux;
}

inline uint64_t pack(const Point4& p, float inv) {
  return pack(p.x(), p.y(), p.z(), inv);
}
inline uint64_t pack(const Point& p, float inv) {
  return pack(p.x(), p.y(), p.z(), inv);
}

// Pack integer coordinates
inline uint64_t pack(int32_t x, int32_t y, int32_t z) {
  x = std::clamp(x, COORD_MIN, COORD_MAX);
  y = std::clamp(y, COORD_MIN, COORD_MAX);
  z = std::clamp(z, COORD_MIN, COORD_MAX);

  uint64_t ux = static_cast<uint64_t>(x + COORD_OFFSET);
  uint64_t uy = static_cast<uint64_t>(y + COORD_OFFSET);
  uint64_t uz = static_cast<uint64_t>(z + COORD_OFFSET);

  return (uz << 42) | (uy << 21) | ux;
}

// Unpack coordinates
inline int32_t unpackX(uint64_t key) {
  return static_cast<int32_t>(key & COORD_MASK) - COORD_OFFSET;
}

inline int32_t unpackY(uint64_t key) {
  return static_cast<int32_t>((key >> 21) & COORD_MASK) - COORD_OFFSET;
}

inline int32_t unpackZ(uint64_t key) {
  return static_cast<int32_t>((key >> 42) & COORD_MASK) - COORD_OFFSET;
}

// Get neighbor key
inline uint64_t neighbor(uint64_t key, int32_t dx, int32_t dy, int32_t dz) {
  return pack(unpackX(key) + dx, unpackY(key) + dy, unpackZ(key) + dz);
}

// Convert key to voxel center
inline Point4 toCenter(uint64_t key, float voxel_size) {
  int32_t ix = unpackX(key);
  int32_t iy = unpackY(key);
  int32_t iz = unpackZ(key);
  return Point4((ix + 0.5f) * voxel_size,
                (iy + 0.5f) * voxel_size,
                (iz + 0.5f) * voxel_size,
                1.0f);
}

inline bool isValid(uint64_t key) {
  return key != INVALID_KEY;
}

// For sort-based voxelization
struct IndexedPoint {
  uint64_t key;
  uint32_t index;
  bool operator<(const IndexedPoint& other) const { return key < other.key; }
};

} // namespace voxel
} // namespace nanopcl

#endif // NANOPCL_CORE_VOXEL_HPP
