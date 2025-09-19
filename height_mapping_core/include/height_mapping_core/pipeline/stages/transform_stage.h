/*
 * transform_stage.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_STAGES_TRANSFORM_STAGE_H
#define HEIGHT_MAPPING_CORE_PIPELINE_STAGES_TRANSFORM_STAGE_H

#include "height_mapping_core/data/transform.h"
#include "height_mapping_core/pipeline/processing_context.h"
#include "height_mapping_core/pipeline/stage.h"
#include <functional>
#include <memory>

namespace height_mapping::core {

// Interface for transform providers
class ITransformProvider {
public:
  virtual ~ITransformProvider() = default;

  virtual std::optional<Transform3D>
  lookupTransform(const std::string &target_frame,
                  const std::string &source_frame, uint64_t timestamp) = 0;

  virtual std::optional<Transform3D>
  lookupLatestTransform(const std::string &target_frame,
                        const std::string &source_frame) = 0;
};

// Stage that transforms point cloud to target ctx
class TransformStage : public PipelineStage<TransformStage> {
public:
  static constexpr const char *STAGE_NAME = "Transform";
  static constexpr const char *STAGE_TYPE = "Coordinate Transform";

  // Constructor with transform provider (using weak_ptr for observation)
  TransformStage(std::weak_ptr<ITransformProvider> provider,
                 const std::string &target_frame)
      : transform_provider_(provider), target_frame_(target_frame) {}

  // Constructor with static transform
  TransformStage(const Transform3D &static_transform)
      : static_transform_(static_transform), use_static_transform_(true) {}

  void processImpl(ProcessingContext &ctx) {
    if (ctx.cloud().empty())
      return;

    Transform3D transform;

    if (use_static_transform_) {
      // Use provided static transform
      transform = static_transform_;
    } else if (auto provider = transform_provider_.lock()) {
      // Safely access provider through lock()
      auto transform_opt = provider->lookupTransform(
          target_frame_, ctx.cloud().frame_id, ctx.cloud().timestamp);

      if (!transform_opt) {
        ctx.setError(getName(), "Failed to get transform from " +
                                      ctx.cloud().frame_id + " to " +
                                      target_frame_ + " at timestamp " +
                                      std::to_string(ctx.cloud().timestamp));
        return;
      }

      transform = *transform_opt;
    } else {
      ctx.setError(getName(), "Transform provider expired or not configured");
      return;
    }

    // Transform the point cloud
    size_t points_before = ctx.cloud().size();

    for (auto &point : ctx.cloud().points) {
      point = transform.apply(point);
    }

    // Update ctx metadata
    ctx.context().sensor_to_map = transform;
    ctx.cloud().frame_id =
        target_frame_; // Update cloud's ctx_id after transform

    // Update statistics
    ctx.set("transform_applied", true);
    ctx.set("points_transformed", points_before);

    // std::cout << "[Transform] Transformed " << points_before
    //           << " points to '" << target_frame_ << "'" << std::endl;
  }

  bool canProcess(const ProcessingContext &ctx) const override {
    // Can process if we have a transform source and non-empty cloud
    return !ctx.cloud().empty() &&
           (use_static_transform_ || !transform_provider_.expired());
  }

  // Configuration
  void setTargetFrame(const std::string &frame) { target_frame_ = frame; }
  std::string getTargetFrame() const { return target_frame_; }

  void setStaticTransform(const Transform3D &transform) {
    static_transform_ = transform;
    use_static_transform_ = true;
  }

private:
  std::weak_ptr<ITransformProvider> transform_provider_;
  std::string target_frame_ = "map";
  Transform3D static_transform_ = Transform3D::Identity();
  bool use_static_transform_ = false;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_STAGES_TRANSFORM_STAGE_H