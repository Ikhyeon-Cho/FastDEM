/*
 * fast_height_filter_stage.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_STAGES_FAST_HEIGHT_FILTER_STAGE_H
#define HEIGHT_MAPPING_CORE_PIPELINE_STAGES_FAST_HEIGHT_FILTER_STAGE_H

#include "height_mapping_core/pipeline/processing_context.h"
#include "height_mapping_core/pipeline/stage.h"
#include <limits>

namespace height_mapping::core {

class FastHeightFilterStage : public PipelineStage<FastHeightFilterStage> {
public:
  static constexpr const char *STAGE_NAME = "FastHeightFilter";
  static constexpr const char *STAGE_TYPE = "Filter";

  FastHeightFilterStage(float min_z = -std::numeric_limits<float>::infinity(),
                        float max_z = std::numeric_limits<float>::infinity())
      : min_z_(min_z), max_z_(max_z) {}

  void initialize() override { initialized_ = true; }

  void processImpl(ProcessingContext &ctx) {
    if (ctx.cloud().empty()) {
      return;
    }

    auto &cloud = ctx.cloud();
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    filtered_cloud.header = cloud.header;
    filtered_cloud.is_dense = cloud.is_dense;

    size_t filtered_count = 0;

    for (const auto &point : cloud.points) {
      if (!point.isFinite()) {
        continue;
      }

      if (point.z >= min_z_ && point.z <= max_z_) {
        filtered_cloud.points.push_back(point);
      } else {
        filtered_count++;
      }
    }

    filtered_cloud.width = filtered_cloud.points.size();
    filtered_cloud.height = 1;

    // Replace with filtered cloud
    ctx.cloud() = filtered_cloud;

    // Update statistics
    ctx.stats().points_filtered = filtered_count;
    ctx.set("height_filter_min", min_z_);
    ctx.set("height_filter_max", max_z_);
  }

  bool canProcess(const ProcessingContext &ctx) const override {
    return !frame.cloud().empty() && initialized_;
  }

  void setHeightLimits(float min_z, float max_z) {
    min_z_ = min_z;
    max_z_ = max_z;
  }

  float getMinHeight() const { return min_z_; }
  float getMaxHeight() const { return max_z_; }

private:
  float min_z_;
  float max_z_;
  bool initialized_ = false;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_STAGES_FAST_HEIGHT_FILTER_STAGE_H