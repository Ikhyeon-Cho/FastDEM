/*
 * move_origin.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ALGORITHMS_MOVE_ORIGIN_H
#define HEIGHT_MAPPING_CORE_ALGORITHMS_MOVE_ORIGIN_H

#include <Eigen/Core>
#include <string>

#include "height_mapping/core/data/height_map.h"

namespace height_mapping::algorithms {

/**
 * @brief Algorithm for updating map origin based on robot position
 *
 * Moves the map origin to keep the robot within the mapped area.
 * Supports multiple update strategies.
 */
class MoveOrigin {
 public:
  enum class UpdateMode {
    CONTINUOUS,  // Always update to robot position
    THRESHOLD,   // Update when robot moves beyond threshold
    CENTERED,    // Keep robot in center region
    FIXED        // Never update (static map)
  };

  /**
   * @brief Configuration parameters for map origin updates
   */
  struct Config {
    UpdateMode update_mode = UpdateMode::CONTINUOUS;
    float update_threshold = 2.0f;    // meters (for THRESHOLD mode)
    float center_region_size = 5.0f;  // meters (for CENTERED mode)
    std::string robot_frame = "base_link";
    std::string map_frame = "map";

    void validate() const {
      if (update_threshold <= 0) {
        throw std::invalid_argument("update_threshold must be positive");
      }
      if (center_region_size <= 0) {
        throw std::invalid_argument("center_region_size must be positive");
      }
    }
  };

  /**
   * @brief Update statistics
   */
  struct Stats {
    bool updated = false;
    std::string update_reason = "none";
    Eigen::Vector3f new_position;
    Eigen::Vector3f old_position;
    float distance_moved = 0.0f;
    size_t total_updates = 0;
    float total_distance = 0.0f;
  };

 private:
  Config config_;
  Eigen::Vector3f last_update_position_{0, 0, 0};
  bool first_update_ = true;
  mutable Stats cumulative_stats_;

 public:
  MoveOrigin() : config_{} { config_.validate(); }

  explicit MoveOrigin(const Config &config) : config_(config) {
    config_.validate();
  }

  void setConfig(const Config &config) {
    config_ = config;
    config_.validate();
  }

  const Config &getConfig() const { return config_; }
  const Stats &getCumulativeStats() const { return cumulative_stats_; }
  void resetStats() {
    cumulative_stats_ = Stats();
    first_update_ = true;
  }

  /**
   * @brief Update map origin based on robot position
   * @param map Height map to update (modified)
   * @param robot_position Current robot position in map frame
   * @return Update statistics
   */
  Stats updateOrigin(HeightMap &map, const Eigen::Vector3f &robot_position);

 private:
  bool shouldUpdate(const Eigen::Vector3f &robot_position, std::string &reason);
};

}  // namespace height_mapping::algorithms

#endif  // HEIGHT_MAPPING_CORE_ALGORITHMS_MOVE_ORIGIN_H