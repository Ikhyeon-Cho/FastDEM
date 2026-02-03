// nanoPCL - Colormap utilities for point cloud visualization
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_VISUALIZATION_RERUN_COLOR_MAP_HPP
#define NANOPCL_VISUALIZATION_RERUN_COLOR_MAP_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#ifdef NANOPCL_USE_RERUN

#include <rerun.hpp>

namespace nanopcl::rr::colormap {

/// Available colormap types
enum class Type {
  TURBO,     ///< Google's Turbo colormap (perceptually uniform, vibrant)
  VIRIDIS,   ///< Matplotlib's Viridis (perceptually uniform, colorblind-safe)
  GRAYSCALE, ///< Simple grayscale
  JET        ///< Classic jet/rainbow (not recommended, but familiar)
};

namespace detail {

/// Turbo colormap polynomial approximation (Google, Apache 2.0)
/// Based on https://gist.github.com/mikhailov-work/0d177465a8151eb6edd1768d51d476c7
inline rerun::Color turbo(float t) {
  t = std::clamp(t, 0.0f, 1.0f);

  // Red channel
  const float r = std::clamp(
      0.13572138f +
          t * (4.61539260f +
               t * (-42.66032258f +
                    t * (132.13108234f +
                         t * (-152.94239396f + t * 59.28637943f)))),
      0.0f, 1.0f);

  // Green channel
  const float g = std::clamp(
      0.09140261f +
          t * (2.19418839f +
               t * (4.84296658f +
                    t * (-14.18503333f + t * (4.27729857f + t * 2.82956604f)))),
      0.0f, 1.0f);

  // Blue channel
  const float b = std::clamp(
      0.10667330f +
          t * (12.64194608f +
               t * (-60.58204836f +
                    t * (110.36276771f +
                         t * (-89.90310912f + t * 27.34824973f)))),
      0.0f, 1.0f);

  return rerun::Color(static_cast<uint8_t>(r * 255),
                      static_cast<uint8_t>(g * 255),
                      static_cast<uint8_t>(b * 255));
}

/// Viridis colormap approximation
inline rerun::Color viridis(float t) {
  t = std::clamp(t, 0.0f, 1.0f);

  // Simplified polynomial approximation
  const float r =
      std::clamp(0.267004f + t * (0.003991f + t * (1.091692f - t * 0.356783f)),
                 0.0f, 1.0f);
  const float g =
      std::clamp(0.004874f + t * (1.421258f + t * (-0.514403f + t * 0.088297f)),
                 0.0f, 1.0f);
  const float b =
      std::clamp(0.329415f + t * (0.283913f + t * (-1.413053f + t * 0.800467f)),
                 0.0f, 1.0f);

  return rerun::Color(static_cast<uint8_t>(r * 255),
                      static_cast<uint8_t>(g * 255),
                      static_cast<uint8_t>(b * 255));
}

/// Jet colormap (classic rainbow)
inline rerun::Color jet(float t) {
  t = std::clamp(t, 0.0f, 1.0f);

  float r, g, b;
  if (t < 0.25f) {
    r = 0.0f;
    g = 4.0f * t;
    b = 1.0f;
  } else if (t < 0.5f) {
    r = 0.0f;
    g = 1.0f;
    b = 1.0f - 4.0f * (t - 0.25f);
  } else if (t < 0.75f) {
    r = 4.0f * (t - 0.5f);
    g = 1.0f;
    b = 0.0f;
  } else {
    r = 1.0f;
    g = 1.0f - 4.0f * (t - 0.75f);
    b = 0.0f;
  }

  return rerun::Color(static_cast<uint8_t>(r * 255),
                      static_cast<uint8_t>(g * 255),
                      static_cast<uint8_t>(b * 255));
}

/// Grayscale
inline rerun::Color grayscale(float t) {
  t = std::clamp(t, 0.0f, 1.0f);
  uint8_t gray = static_cast<uint8_t>(t * 255);
  return rerun::Color(gray, gray, gray);
}

} // namespace detail

/// Map a normalized value [0, 1] to a color
inline rerun::Color map(float t, Type type = Type::TURBO) {
  switch (type) {
  case Type::TURBO:
    return detail::turbo(t);
  case Type::VIRIDIS:
    return detail::viridis(t);
  case Type::JET:
    return detail::jet(t);
  case Type::GRAYSCALE:
    return detail::grayscale(t);
  default:
    return detail::turbo(t);
  }
}

/// Apply colormap to a vector of scalar values
/// @param values Input scalar values
/// @param type Colormap type to use
/// @param min_val Minimum value for normalization (NAN for auto)
/// @param max_val Maximum value for normalization (NAN for auto)
/// @return Vector of colors corresponding to each input value
inline std::vector<rerun::Color> apply(const std::vector<float>& values,
                                       Type type = Type::TURBO,
                                       float min_val = NAN,
                                       float max_val = NAN) {
  if (values.empty()) {
    return {};
  }

  // Auto-range if not specified
  if (std::isnan(min_val)) {
    min_val = *std::min_element(values.begin(), values.end());
  }
  if (std::isnan(max_val)) {
    max_val = *std::max_element(values.begin(), values.end());
  }

  const float range = (max_val - min_val) > 1e-6f ? (max_val - min_val) : 1.0f;

  std::vector<rerun::Color> colors;
  colors.reserve(values.size());

  for (float v : values) {
    float t = (v - min_val) / range;
    colors.push_back(map(t, type));
  }

  return colors;
}

/// Apply colormap to uint16_t values (e.g., ring IDs)
inline std::vector<rerun::Color> apply(const std::vector<uint16_t>& values,
                                       Type type = Type::TURBO,
                                       uint16_t min_val = 0,
                                       uint16_t max_val = 0) {
  if (values.empty()) {
    return {};
  }

  // Auto-range if not specified
  if (max_val == 0) {
    max_val = *std::max_element(values.begin(), values.end());
  }

  const float range =
      (max_val - min_val) > 0 ? static_cast<float>(max_val - min_val) : 1.0f;

  std::vector<rerun::Color> colors;
  colors.reserve(values.size());

  for (uint16_t v : values) {
    float t = static_cast<float>(v - min_val) / range;
    colors.push_back(map(t, type));
  }

  return colors;
}

} // namespace nanopcl::rr::colormap

#endif // NANOPCL_USE_RERUN

#endif // NANOPCL_VISUALIZATION_RERUN_COLOR_MAP_HPP
