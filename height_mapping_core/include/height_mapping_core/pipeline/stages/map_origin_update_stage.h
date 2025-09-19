/*
 * map_origin_update_stage.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_STAGES_MAP_ORIGIN_UPDATE_STAGE_H
#define HEIGHT_MAPPING_CORE_PIPELINE_STAGES_MAP_ORIGIN_UPDATE_STAGE_H

#include "height_mapping_core/pipeline/processing_context.h"
#include "height_mapping_core/pipeline/stage.h"
#include "height_mapping_core/pipeline/stages/transform_stage.h" // For ITransformProvider
#include <grid_map_core/GridMap.hpp>
#include <memory>

namespace height_mapping::core {

/**
 * @brief Pipeline stage that updates the map origin to follow the robot
 *
 * This stage moves the height map origin to keep the robot centered,
 * allowing for continuous mapping in large environments.
 */
class MapOriginUpdateStage : public PipelineStage<MapOriginUpdateStage> {
public:
  static constexpr const char *STAGE_NAME = "MapOriginUpdate";
  static constexpr const char *STAGE_TYPE = "Map Management";

  enum class UpdateMode {
    CONTINUOUS, // Always update to robot position
    THRESHOLD,  // Update only when moved beyond threshold
    CENTERED    // Keep robot in center of map
  };

  /**
   * @brief Constructor with transform provider
   * @param tf_provider Provider for getting robot transforms
   * @param robot_frame Frame ID of the robot base
   * @param map_frame Frame ID of the map
   * @param mode Update mode (continuous, threshold, or centered)
   * @param update_threshold Minimum distance before updating (only for
   * THRESHOLD mode)
   */
  MapOriginUpdateStage(std::weak_ptr<ITransformProvider> tf_provider,
                       const std::string &robot_frame = "base_link",
                       const std::string &map_frame = "map",
                       UpdateMode mode = UpdateMode::CONTINUOUS,
                       float update_threshold = 1.0f)
      : tf_provider_(tf_provider), robot_frame_(robot_frame),
        map_frame_(map_frame), update_mode_(mode),
        update_threshold_(update_threshold) {

    if (tf_provider_.expired()) {
      throw std::invalid_argument(
          "Transform provider is required for MapOriginUpdateStage");
    }
  }

  void initialize() override {
    last_update_position_ = grid_map::Position(0, 0);
  }

  void processImpl(ProcessingContext &ctx) {
    // Check if map pointer exists
    auto map_ptr = ctx.context().map;
    if (!map_ptr) {
      // std::cout << "[MapOriginUpdate] Map pointer is null" << std::endl;
      return;
    }

    // Map can be empty (no data) but still need to move origin
    // The map just needs to be initialized with dimensions

    // Get current robot position
    // Use latest transform for robot position (not cloud timestamp)
    // This ensures we get the most recent robot position
    auto provider = tf_provider_.lock();
    if (!provider) {
      ctx.set("map_origin_error",
                std::string("Transform provider expired"));
      return;
    }

    auto transform = provider->lookupLatestTransform(map_frame_, robot_frame_);
    if (!transform) {
      // Store warning in metadata
      ctx.set("map_origin_warning",
                std::string("Could not get robot transform"));
      // std::cout << "[MapOriginUpdate] Could not get transform from "
      //           << robot_frame_ << " to " << map_frame_ << std::endl;
      return;
    }

    grid_map::Position robot_position(transform->x(), transform->y());
    bool should_update = false;

    switch (update_mode_) {
    case UpdateMode::CONTINUOUS:
      // Always update to robot position
      should_update = true;
      break;

    case UpdateMode::THRESHOLD:
      // Update only if moved beyond threshold
      {
        float distance_moved = (robot_position - last_update_position_).norm();
        should_update = (distance_moved >= update_threshold_);
      }
      break;

    case UpdateMode::CENTERED:
      // Keep robot within central region of map
      {
        // Get map bounds
        auto map_size = map_ptr->getLength();
        float max_offset =
            std::min(map_size.x(), map_size.y()) * 0.25f; // 25% from center

        // Check if robot is near edge
        auto current_origin = map_ptr->getPosition();
        float offset_from_origin = (robot_position - current_origin).norm();
        should_update = (offset_from_origin > max_offset);
      }
      break;
    }

    if (should_update) {
      // Move the map origin to robot position
      // std::cout << "[MapOriginUpdate] Moving map origin to robot position: ("
      //           << robot_position.x() << ", " << robot_position.y() << ")" <<
      //           std::endl;
      map_ptr->move(robot_position);
      last_update_position_ = robot_position;

      // Log the update
      ctx.set("map_origin_x", robot_position.x());
      ctx.set("map_origin_y", robot_position.y());
      ctx.set("map_origin_updated", true);

      if (update_count_++ % 10 == 0) { // Log every 10th update to reduce spam
        ctx.set("map_origin_info",
                  std::string("Map origin updated to: (") +
                      std::to_string(robot_position.x()) + ", " +
                      std::to_string(robot_position.y()) + ")");
      }
    } else {
      ctx.set("map_origin_updated", false);
      // if (update_count_++ % 30 == 0) {  // Log occasionally why not updating
      //     std::cout << "[MapOriginUpdate] Not updating - mode: " <<
      //     static_cast<int>(update_mode_)
      //               << ", robot at (" << robot_position.x() << ", " <<
      //               robot_position.y()
      //               << "), last at (" << last_update_position_.x() << ", "
      //               << last_update_position_.y() << ")" << std::endl;
      // }
    }
  }

  void reset() {
    last_update_position_ = grid_map::Position(0, 0);
    update_count_ = 0;
  }

  // Configuration
  void setUpdateMode(UpdateMode mode) { update_mode_ = mode; }

  UpdateMode getUpdateMode() const { return update_mode_; }

  void setUpdateThreshold(float threshold) { update_threshold_ = threshold; }

  float getUpdateThreshold() const { return update_threshold_; }

  void setRobotFrame(const std::string &frame) { robot_frame_ = frame; }

  void setMapFrame(const std::string &frame) { map_frame_ = frame; }

private:
  std::weak_ptr<ITransformProvider> tf_provider_;
  std::string robot_frame_;
  std::string map_frame_;
  UpdateMode update_mode_;
  float update_threshold_; // Minimum distance before updating origin (for
                           // THRESHOLD mode)
  grid_map::Position last_update_position_;
  size_t update_count_ = 0; // Counter for logging
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_STAGES_MAP_ORIGIN_UPDATE_STAGE_H