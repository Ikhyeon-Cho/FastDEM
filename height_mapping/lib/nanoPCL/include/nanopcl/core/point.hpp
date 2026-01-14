// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_CORE_POINT_HPP
#define NANOPCL_CORE_POINT_HPP

#include <Eigen/Core>
#include <array>
#include <cstdint>

namespace npcl {

/// 3D point with float precision
using Point = Eigen::Vector3f;

/**
 * @brief Strong Types for Point Attributes
 *
 * These structs serve two roles:
 * 1. Type-safe tags for variadic add() (API)
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
 * @brief Semantic label for point cloud segmentation (32-bit packed)
 *
 * Efficiently stores both semantic class and instance ID in a single 32-bit integer.
 * Compatible with SemanticKITTI and NuScenes formats.
 *
 * Bit Layout:
 * - [0-15]  Lower 16 bits: Semantic Class (e.g., 10=Car, 40=Road)
 * - [16-31] Upper 16 bits: Instance ID (distinguishes individual objects)
 *
 * @note sizeof(Label) == sizeof(uint32_t) for memory efficiency
 *
 * @code
 *   // Create label for Car (class 10), instance 5
 *   Label l(SemanticClass(10), InstanceId(5));
 *   
 *   uint16_t cls = l.semanticClass(); // 10
 *   uint16_t id = l.instanceId();     // 5
 * @endcode
 */
struct Label {
  uint32_t val;

  // --- Constructors ---
  constexpr Label() : val(0) {}
  explicit constexpr Label(uint32_t raw) : val(raw) {}

  constexpr Label(SemanticClass cls, InstanceId inst = InstanceId(0))
      : val((static_cast<uint32_t>(inst.val) << 16) | cls.val) {}

  // --- Accessors ---
  
  /// Get lower 16 bits (semantic class)
  constexpr uint16_t semanticClass() const {
    return static_cast<uint16_t>(val & 0xFFFF);
  }
  
  /// Get upper 16 bits (instance ID)
  constexpr uint16_t instanceId() const {
    return static_cast<uint16_t>(val >> 16);
  }

  // --- Mutators ---
  
  /// Set lower 16 bits (semantic class)
  constexpr void setSemanticClass(uint16_t cls) {
    val = (val & 0xFFFF0000) | cls;
  }
  
  /// Set upper 16 bits (instance ID)
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

// =============================================================================
// Point DTOs (Data Transfer Objects)
// =============================================================================
// PCL-style convenience types for add() operations.
// These are for DATA TRANSFER only - internal storage remains SoA.
//
// Usage: cloud.add(PointXYZI{1.0f, 2.0f, 3.0f, 0.5f});

/// Point with intensity (LiDAR basic)
struct PointXYZI {
  float x, y, z;
  float intensity;

  constexpr PointXYZI(float x_, float y_, float z_, float i_)
      : x(x_), y(y_), z(z_), intensity(i_) {}
};

/// Point with intensity and ring (multi-beam LiDAR)
struct PointXYZIR {
  float x, y, z;
  float intensity;
  uint16_t ring;

  constexpr PointXYZIR(float x_, float y_, float z_, float i_, uint16_t r_)
      : x(x_), y(y_), z(z_), intensity(i_), ring(r_) {}
};

/// Point with intensity and time (for deskewing)
struct PointXYZIT {
  float x, y, z;
  float intensity;
  float time;

  constexpr PointXYZIT(float x_, float y_, float z_, float i_, float t_)
      : x(x_), y(y_), z(z_), intensity(i_), time(t_) {}
};

/// Point with intensity, ring, and time (full LiDAR)
struct PointXYZIRT {
  float x, y, z;
  float intensity;
  uint16_t ring;
  float time;

  constexpr PointXYZIRT(float x_, float y_, float z_, float i_, uint16_t r_,
                        float t_)
      : x(x_), y(y_), z(z_), intensity(i_), ring(r_), time(t_) {}
};

/// Point with RGB color (RGB-D cameras)
struct PointXYZRGB {
  float x, y, z;
  uint8_t r, g, b;

  constexpr PointXYZRGB(float x_, float y_, float z_, uint8_t r_, uint8_t g_,
                        uint8_t b_)
      : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_) {}
};

/// Point with color and normal (RGB surface reconstruction)
struct PointXYZRGBN {
  float x, y, z;
  uint8_t r, g, b;
  float nx, ny, nz;

  constexpr PointXYZRGBN(float x_, float y_, float z_, uint8_t r_, uint8_t g_,
                         uint8_t b_, float nx_, float ny_, float nz_)
      : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_), nx(nx_), ny(ny_), nz(nz_) {}
};

/// Point with color and label (RGB segmentation)
struct PointXYZRGBL {
  float x, y, z;
  uint8_t r, g, b;
  uint32_t label;

  constexpr PointXYZRGBL(float x_, float y_, float z_, uint8_t r_, uint8_t g_,
                         uint8_t b_, uint32_t l_)
      : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_), label(l_) {}
};

/// Point with semantic label (segmentation)
struct PointXYZL {
  float x, y, z;
  uint32_t label;

  constexpr PointXYZL(float x_, float y_, float z_, uint32_t l_)
      : x(x_), y(y_), z(z_), label(l_) {}
};

/// Point with intensity and label (segmentation + LiDAR)
struct PointXYZIL {
  float x, y, z;
  float intensity;
  uint32_t label;

  constexpr PointXYZIL(float x_, float y_, float z_, float i_, uint32_t l_)
      : x(x_), y(y_), z(z_), intensity(i_), label(l_) {}
};

/// Point with normal vector (surface reconstruction)
struct PointXYZN {
  float x, y, z;
  float nx, ny, nz;

  constexpr PointXYZN(float x_, float y_, float z_, float nx_, float ny_,
                      float nz_)
      : x(x_), y(y_), z(z_), nx(nx_), ny(ny_), nz(nz_) {}
};

/// Point with intensity and normal (feature extraction)
struct PointXYZIN {
  float x, y, z;
  float intensity;
  float nx, ny, nz;

  constexpr PointXYZIN(float x_, float y_, float z_, float i_, float nx_,
                       float ny_, float nz_)
      : x(x_), y(y_), z(z_), intensity(i_), nx(nx_), ny(ny_), nz(nz_) {}
};

}  // namespace npcl

#endif  // NANOPCL_CORE_POINT_HPP
