/*
 * configs.h
 *
 * Configuration structures for height mapping algorithms.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_ALGORITHMS_CONFIGS_H
#define HEIGHT_MAPPING_ALGORITHMS_CONFIGS_H

#include <stdexcept>
#include <string>

// Forward declaration for YAML loading
namespace YAML { class Node; }

namespace height_mapping::algorithms {

// =============================================================================
// Persistence (Obstacle Durability)
// =============================================================================

struct PersistenceConfig {
  float max_count = 30.0f;  // Max durability (3 sec @ 10Hz)
  float step_add = 3.0f;    // Hit: increment per observation
  float step_sub = 1.0f;    // Miss: decrement per ray pass

  /// Create from YAML node (implementation in configs.cpp)
  static PersistenceConfig fromYaml(const YAML::Node& node);

  void validate() const {
    if (max_count <= 0) {
      throw std::invalid_argument("max_count must be positive");
    }
    if (step_add <= 0) {
      throw std::invalid_argument("step_add must be positive");
    }
    if (step_sub <= 0) {
      throw std::invalid_argument("step_sub must be positive");
    }
  }
};

// =============================================================================
// Raycasting
// =============================================================================

struct RaycastingConfig {
  bool enable_correction = true;
  float correction_threshold = 0.05f;  // [m] Ghost clearing threshold
  float min_ray_distance = 0.5f;       // [m] Skip rays shorter than this
  PersistenceConfig persistence;       // Obstacle durability settings

  /// Create from YAML node (implementation in configs.cpp)
  static RaycastingConfig fromYaml(const YAML::Node& node);

  void validate() const {
    if (correction_threshold <= 0) {
      throw std::invalid_argument("correction_threshold must be positive");
    }
    if (min_ray_distance <= 0) {
      throw std::invalid_argument("min_ray_distance must be positive");
    }
    persistence.validate();
  }
};

// =============================================================================
// Inpainting
// =============================================================================

struct InpaintingConfig {
  bool enabled = false;
  int max_iterations = 3;       // Usually 1-3 sufficient for small holes
  int min_valid_neighbors = 2;  // Minimum neighbors to interpolate

  /// Create from YAML node (implementation in configs.cpp)
  static InpaintingConfig fromYaml(const YAML::Node& node);

  void validate() const {
    if (max_iterations < 1) {
      throw std::invalid_argument("max_iterations must be at least 1");
    }
    if (min_valid_neighbors < 1 || min_valid_neighbors > 8) {
      throw std::invalid_argument("min_valid_neighbors must be in [1, 8]");
    }
  }
};

}  // namespace height_mapping::algorithms

#endif  // HEIGHT_MAPPING_ALGORITHMS_CONFIGS_H
