// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_TYPES_HPP
#define NANOPCL_CORE_TYPES_HPP

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cstdint>
#include <vector>

namespace nanopcl {

// Aligned vector for Eigen types (SIMD)
template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

// Point types
using Point = Eigen::Vector3f;      // 3D point (user-facing)
using Point4 = Eigen::Vector4f;     // 4D point (internal storage, SIMD aligned)
using Normal4 = Eigen::Vector4f;    // (nx, ny, nz, w=0)
using Covariance = Eigen::Matrix3f; // 3x3 covariance matrix

// Strong types for type-safe add() overloads
struct Intensity {
  float val;
  explicit constexpr Intensity(float v = 0.0f)
      : val(v) {}
  constexpr operator float() const { return val; }
};

struct Time {
  float val;
  explicit constexpr Time(float v = 0.0f)
      : val(v) {}
  constexpr operator float() const { return val; }
};

struct Ring {
  uint16_t val;
  explicit constexpr Ring(uint16_t v = 0)
      : val(v) {}
  constexpr operator uint16_t() const { return val; }
};

struct Color {
  uint8_t r, g, b;
  constexpr Color()
      : r(0), g(0), b(0) {}
  constexpr Color(uint8_t r_, uint8_t g_, uint8_t b_)
      : r(r_), g(g_), b(b_) {}
};

struct Label {
  uint32_t val;
  constexpr Label()
      : val(0) {}
  explicit constexpr Label(uint32_t v)
      : val(v) {}
  constexpr operator uint32_t() const { return val; }
};

// Timestamp helpers (nanoseconds <-> seconds)
inline double toSec(uint64_t ns) {
  return ns * 1e-9;
}
inline uint64_t fromSec(double s) {
  return static_cast<uint64_t>(s * 1e9);
}

// Index range for range-based for loops
class IndexRange {
  size_t n_;

public:
  struct Iterator {
    size_t i;
    constexpr size_t operator*() const noexcept { return i; }
    constexpr Iterator& operator++() noexcept {
      ++i;
      return *this;
    }
    constexpr bool operator!=(Iterator o) const noexcept { return i != o.i; }
  };

  constexpr explicit IndexRange(size_t n) noexcept
      : n_(n) {}
  constexpr Iterator begin() const noexcept { return {0}; }
  constexpr Iterator end() const noexcept { return {n_}; }
  constexpr size_t size() const noexcept { return n_; }
};

} // namespace nanopcl

#endif // NANOPCL_CORE_TYPES_HPP
