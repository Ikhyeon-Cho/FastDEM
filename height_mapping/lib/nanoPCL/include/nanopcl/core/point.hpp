// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_POINT_HPP
#define NANOPCL_CORE_POINT_HPP

#include <Eigen/Core>
#include <array>
#include <cstdint>

namespace nanopcl {

/// 3D point with float precision
using Point = Eigen::Vector3f;

/**
 * @brief Strong Types for Point Attributes
 *
 * These structs serve two roles:
 * 1. Type-safe tags for variadic push_back (API)
 * 2. Data storage types (Implementation) - Zero overhead wrappers
 */

struct Intensity {
  float val;
  explicit constexpr Intensity(float v) : val(v) {}
};

struct Time {
  float val;
  explicit constexpr Time(float v) : val(v) {}
};

struct Ring {
  uint16_t val;
  explicit constexpr Ring(uint16_t v) : val(v) {}
};

struct Normal {
  Eigen::Vector3f val;
  explicit Normal(const Eigen::Vector3f& v) : val(v) {}
  Normal(float nx, float ny, float nz) : val(nx, ny, nz) {}
};

struct Color {
  std::array<uint8_t, 3> rgb{0, 0, 0};

  // Constructors
  constexpr Color() = default;
  constexpr Color(uint8_t r, uint8_t g, uint8_t b) : rgb{r, g, b} {}
  explicit constexpr Color(const std::array<uint8_t, 3>& v) : rgb(v) {}

  // Factory methods
  static constexpr Color fromHex(uint32_t hex) {
    return Color((hex >> 16) & 0xFF, (hex >> 8) & 0xFF, hex & 0xFF);
  }

  static constexpr Color fromNormalized(float r, float g, float b) {
    return Color(static_cast<uint8_t>(r * 255), static_cast<uint8_t>(g * 255),
                 static_cast<uint8_t>(b * 255));
  }

  // Predefined colors
  static constexpr Color Black() { return Color(0, 0, 0); }
  static constexpr Color White() { return Color(255, 255, 255); }
  static constexpr Color Red() { return Color(255, 0, 0); }
  static constexpr Color Green() { return Color(0, 255, 0); }
  static constexpr Color Blue() { return Color(0, 0, 255); }
  static constexpr Color Yellow() { return Color(255, 255, 0); }
  static constexpr Color Cyan() { return Color(0, 255, 255); }
  static constexpr Color Magenta() { return Color(255, 0, 255); }

  // Conversion
  constexpr uint32_t toHex() const {
    return (static_cast<uint32_t>(rgb[0]) << 16) |
           (static_cast<uint32_t>(rgb[1]) << 8) |  //
           static_cast<uint32_t>(rgb[2]);
  }

  // Accessors
  constexpr uint8_t r() const { return rgb[0]; }
  constexpr uint8_t g() const { return rgb[1]; }
  constexpr uint8_t b() const { return rgb[2]; }

  constexpr uint8_t& r() { return rgb[0]; }
  constexpr uint8_t& g() { return rgb[1]; }
  constexpr uint8_t& b() { return rgb[2]; }

  // Array access
  constexpr uint8_t& operator[](size_t i) { return rgb[i]; }
  constexpr uint8_t operator[](size_t i) const { return rgb[i]; }

  // Comparison (element-wise for C++17 constexpr compatibility)
  constexpr bool operator==(const Color& other) const {
    return rgb[0] == other.rgb[0] &&  //
           rgb[1] == other.rgb[1] &&  //
           rgb[2] == other.rgb[2];
  }
  constexpr bool operator!=(const Color& other) const {
    return !(*this == other);
  }
};

// =============================================================================
// Label (Semantic Segmentation)
// =============================================================================

/// Strong type for semantic class construction
struct SemanticClass {
  uint16_t val;
  explicit constexpr SemanticClass(uint16_t v) : val(v) {}
};

/// Strong type for instance ID construction
struct InstanceId {
  uint16_t val;
  explicit constexpr InstanceId(uint16_t v = 0) : val(v) {}
};

/**
 * @brief Semantic label for point cloud segmentation
 *
 * Follows SemanticKITTI/NuScenes format:
 * - Lower 16 bits: semantic class (e.g., 10=car, 11=bicycle)
 * - Upper 16 bits: instance ID (distinguishes objects of same class)
 *
 * @note sizeof(Label) == sizeof(uint32_t) for memory efficiency
 */
struct Label {
  uint32_t val;

  // --- Constructors ---
  constexpr Label() : val(0) {}
  explicit constexpr Label(uint32_t raw) : val(raw) {}

  constexpr Label(SemanticClass cls, InstanceId inst = InstanceId(0))
      : val((static_cast<uint32_t>(inst.val) << 16) | cls.val) {}

  // --- Accessors ---
  constexpr uint16_t semanticClass() const {
    return static_cast<uint16_t>(val & 0xFFFF);
  }
  constexpr uint16_t instanceId() const {
    return static_cast<uint16_t>(val >> 16);
  }

  // --- Mutators ---
  constexpr void setSemanticClass(uint16_t cls) {
    val = (val & 0xFFFF0000) | cls;
  }
  constexpr void setInstanceId(uint16_t inst) {
    val = (val & 0x0000FFFF) | (static_cast<uint32_t>(inst) << 16);
  }

  // --- Comparison ---
  constexpr bool operator==(const Label& other) const {
    return val == other.val;
  }
  constexpr bool operator!=(const Label& other) const {
    return val != other.val;
  }
};

static_assert(sizeof(Label) == sizeof(uint32_t), "Label must be 4 bytes");

}  // namespace nanopcl

#endif  // NANOPCL_CORE_POINT_HPP
