/*
 * passthrough_filter_stage.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_STAGES_PASSTHROUGH_FILTER_STAGE_H
#define HEIGHT_MAPPING_CORE_PIPELINE_STAGES_PASSTHROUGH_FILTER_STAGE_H

#include "height_mapping_core/pipeline/processing_context.h"
#include "height_mapping_core/pipeline/stage.h"

namespace height_mapping::core {

// Stage that filters points outside specified bounds
class PassthroughFilterStage : public PipelineStage<PassthroughFilterStage> {
public:
  static constexpr const char *STAGE_NAME = "PassthroughFilter";
  static constexpr const char *STAGE_TYPE = "Point Cloud Filter";

  struct Limits {
    float min = -std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::max();
    bool enabled = false;

    bool isInRange(float value) const {
      return !enabled || (value >= min && value <= max);
    }
  };

  PassthroughFilterStage() = default;

  void processImpl(ProcessingContext &ctx) {
    if (ctx.cloud().empty())
      return;

    size_t points_before = ctx.cloud().size();
    // std::cout << "[PassthroughFilter] Input: " << points_before << " points"
    // << std::endl;

    // Filter points
    ctx.cloud().filterInPlace([this](const Point3D &point) {
      // Check each axis
      if (!x_limits_.isInRange(point.x))
        return false;
      if (!y_limits_.isInRange(point.y))
        return false;
      if (!z_limits_.isInRange(point.z))
        return false;

      // Check distance from origin if enabled
      if (distance_limits_.enabled) {
        float distance = point.norm();
        if (!distance_limits_.isInRange(distance))
          return false;
      }

      return true;
    });

    size_t points_after = ctx.cloud().size();
    size_t points_removed = points_before - points_after;

    // std::cout << "[PassthroughFilter] Output: " << points_after << " points"
    //           << " (removed " << points_removed << ")" << std::endl;

    if (points_after == 0) {
      // std::cout << "[PassthroughFilter] WARNING: All points filtered out!" <<
      // std::endl; std::cout << "  X limits: " << (x_limits_.enabled ?
      //     std::to_string(x_limits_.min) + " to " +
      //     std::to_string(x_limits_.max) : "disabled") << std::endl;
      // std::cout << "  Y limits: " << (y_limits_.enabled ?
      //     std::to_string(y_limits_.min) + " to " +
      //     std::to_string(y_limits_.max) : "disabled") << std::endl;
      // std::cout << "  Z limits: " << (z_limits_.enabled ?
      //     std::to_string(z_limits_.min) + " to " +
      //     std::to_string(z_limits_.max) : "disabled") << std::endl;
    }

    // Update statistics
    ctx.stats().points_filtered += points_removed;
    ctx.set("passthrough_points_removed", points_removed);
    ctx.set("passthrough_filter_ratio",
              static_cast<float>(points_after) / points_before);
  }

  bool canProcess(const ProcessingContext &ctx) const override {
    // Can process if any filter is enabled
    return !frame.cloud().empty() &&
           (x_limits_.enabled || y_limits_.enabled || z_limits_.enabled ||
            distance_limits_.enabled);
  }

  // Configuration methods
  void setXLimits(float min, float max) {
    x_limits_.min = min;
    x_limits_.max = max;
    x_limits_.enabled = true;
  }

  void setYLimits(float min, float max) {
    y_limits_.min = min;
    y_limits_.max = max;
    y_limits_.enabled = true;
  }

  void setZLimits(float min, float max) {
    z_limits_.min = min;
    z_limits_.max = max;
    z_limits_.enabled = true;
  }

  void setDistanceLimits(float min, float max) {
    distance_limits_.min = min;
    distance_limits_.max = max;
    distance_limits_.enabled = true;
  }

  void disableXLimits() { x_limits_.enabled = false; }
  void disableYLimits() { y_limits_.enabled = false; }
  void disableZLimits() { z_limits_.enabled = false; }
  void disableDistanceLimits() { distance_limits_.enabled = false; }

  // Getters
  const Limits &getXLimits() const { return x_limits_; }
  const Limits &getYLimits() const { return y_limits_; }
  const Limits &getZLimits() const { return z_limits_; }
  const Limits &getDistanceLimits() const { return distance_limits_; }

private:
  Limits x_limits_;
  Limits y_limits_;
  Limits z_limits_;
  Limits distance_limits_;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_STAGES_PASSTHROUGH_FILTER_STAGE_H