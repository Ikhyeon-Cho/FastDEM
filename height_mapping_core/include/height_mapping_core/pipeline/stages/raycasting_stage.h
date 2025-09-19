/*
 * raycasting_stage.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_STAGES_RAYCASTING_STAGE_H
#define HEIGHT_MAPPING_CORE_PIPELINE_STAGES_RAYCASTING_STAGE_H

#include "height_mapping_core/layers.h"
#include "height_mapping_core/pipeline/processing_context.h"
#include "height_mapping_core/pipeline/stage.h"
#include <Eigen/Core>

namespace height_mapping::core {

class RaycastingStage : public PipelineStage<RaycastingStage> {
public:
  static constexpr const char *STAGE_NAME = "Raycasting";
  static constexpr const char *STAGE_TYPE = "Map Correction";

  explicit RaycastingStage(float correction_threshold = 0.02f)
      : correction_threshold_(correction_threshold) {}

  void initialize() override { initialized_ = true; }

  void processImpl(ProcessingContext &ctx) {
    if (ctx.cloud().empty()) {
      return;
    }

    // Get sensor origin from ctx metadata
    auto sensor_origin = ctx.getOrDefault<Eigen::Vector3f>(
        "sensor_origin", Eigen::Vector3f(0, 0, 0));

    auto &map = ctx.map();
    if (!map.isInitialized()) {
      return;
    }

    correctWithRaycasting(map, ctx.cloud(), sensor_origin);

    ctx.stats().processing_time_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - ctx.stats().timestamp)
            .count();
  }

  bool canProcess(const ProcessingContext &ctx) const override {
    return !frame.cloud().empty() && initialized_;
  }

  void setCorrectionThreshold(float threshold) {
    correction_threshold_ = threshold;
  }

private:
  void correctWithRaycasting(height_map::HeightMap &map,
                             const pcl::PointCloud<pcl::PointXYZ> &cloud,
                             const Eigen::Vector3f &sensor_origin) {

    auto &height_matrix = map.getHeightMatrix();
    auto &max_height_matrix = map.getHeightMaxMatrix();
    auto &variance_matrix = map.getHeightVarianceMatrix();
    auto &measurement_count = map.getMeasurementCountMatrix();

    // Add raycasting visualization layer
    map.addLayer(layers::Scan::RAY_CASTING);
    map.clear(layers::Scan::RAY_CASTING);
    auto &raycasting_matrix = map.get(layers::Scan::RAY_CASTING);

    // Add scan height layer
    map.addLayer(layers::Scan::SCAN_HEIGHT);
    map.clear(layers::Scan::SCAN_HEIGHT);
    auto &scan_height_matrix = map.get(layers::Scan::SCAN_HEIGHT);

    // Record current scan heights
    grid_map::Index measured_index;
    grid_map::Position measured_position;

    for (const auto &point : cloud.points) {
      measured_position << point.x, point.y;
      if (!map.getIndex(measured_position, measured_index))
        continue;
      scan_height_matrix(measured_index(0), measured_index(1)) = point.z;
    }

    // Raycasting loop
    const float sampling_step = map.getResolution();

    for (const auto &point : cloud.points) {
      // Create ray from sensor to measured point
      Eigen::Vector3f ray_dir(point.x - sensor_origin.x(),
                              point.y - sensor_origin.y(),
                              point.z - sensor_origin.z());
      float ray_length = ray_dir.norm();
      ray_dir.normalize();

      // Trace along ray
      for (float t = 0; t < ray_length - sampling_step; t += sampling_step) {
        Eigen::Vector3f point_on_ray = sensor_origin + ray_dir * t;

        grid_map::Position check_position(point_on_ray.x(), point_on_ray.y());
        grid_map::Index check_index;

        if (!map.getIndex(check_position, check_index))
          continue;

        // Check for obstacles blocking the ray
        auto &scan_height = scan_height_matrix(check_index(0), check_index(1));
        if (std::isfinite(scan_height) && scan_height > point_on_ray.z() + 0.1)
          break;

        // Get map values at ray point
        auto &map_height = height_matrix(check_index(0), check_index(1));
        auto &map_variance = variance_matrix(check_index(0), check_index(1));
        auto &ray_height = raycasting_matrix(check_index(0), check_index(1));
        auto &n_points = measurement_count(check_index(0), check_index(1));

        // Update ray visualization
        if (!std::isfinite(ray_height))
          ray_height = point_on_ray.z();
        else if (ray_height > point_on_ray.z())
          ray_height = point_on_ray.z();

        // Correct height if map is above ray
        if (map_height > point_on_ray.z() + correction_threshold_) {
          map_variance += (map_height - point_on_ray.z());
          n_points = 1;
          map_height = point_on_ray.z() + correction_threshold_;
        }
      }
    }
  }

  float correction_threshold_;
  bool initialized_ = false;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_STAGES_RAYCASTING_STAGE_H