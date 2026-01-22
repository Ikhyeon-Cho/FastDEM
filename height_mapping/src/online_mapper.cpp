/*
 * online_mapper.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/api/online_mapper.h"

#include <spdlog/spdlog.h>

#include "height_mapping/api/transform_providers.h"

namespace height_mapping {

OnlineMapper::OnlineMapper(const Config& config,
                           std::shared_ptr<IExtrinsicsProvider> extrinsics,
                           std::shared_ptr<IRobotPoseProvider> pose)
    : extrinsics_(std::move(extrinsics)),
      pose_(std::move(pose)),
      mapper_(std::make_unique<Mapper>(config)) {
  if (!extrinsics_) {
    throw std::invalid_argument("Extrinsics provider is required");
  }
  if (!pose_) {
    throw std::invalid_argument("Robot pose provider is required");
  }

  spdlog::debug("[OnlineMapper] Initialized with Mapper composition");
}

OnlineMapper::~OnlineMapper() = default;

void OnlineMapper::integrate(std::shared_ptr<PointCloud> cloud) {
  // Validate input
  if (!cloud || cloud->empty()) {
    spdlog::warn("[OnlineMapper] Received empty or null cloud. Skipping.");
    return;
  }
  if (cloud->frameId().empty()) {
    spdlog::error("[OnlineMapper] Input cloud has no frameId. Skipping.");
    return;
  }

  // Resolve transforms
  auto T_base_sensor_opt = extrinsics_->getExtrinsic(cloud->frameId());
  if (!T_base_sensor_opt) {
    spdlog::warn("[OnlineMapper] Extrinsic not available for '{}'. Skipping.",
                 cloud->frameId());
    return;
  }

  auto T_map_base_opt = pose_->getRobotPoseAt(cloud->timestamp());
  if (!T_map_base_opt) {
    spdlog::warn("[OnlineMapper] Robot pose not available at {}. Skipping.",
                 cloud->timestamp());
    return;
  }

  const auto& T_base_sensor = *T_base_sensor_opt;
  const auto& T_map_base = *T_map_base_opt;

  // Delegate preprocessing to Mapper (lock-free)
  auto data = mapper_->processScan(*cloud, T_base_sensor, T_map_base);
  if (data.points.empty()) return;

  // Invoke callback with preprocessed cloud (between preprocess and update)
  if (processed_scan_callback_) {
    processed_scan_callback_(data.points);
  }

  // Delegate map update to Mapper (acquires lock internally)
  mapper_->updateMap(data);
}

const HeightMap& OnlineMapper::map() const { return mapper_->map(); }

void OnlineMapper::reset() { mapper_->reset(); }

void OnlineMapper::moveMapOrigin(const Eigen::Vector2f& position) {
  mapper_->moveMapOrigin(position);
}

void OnlineMapper::setProcessedScanCallback(ProcessedScanCallback callback) {
  processed_scan_callback_ = std::move(callback);
}

}  // namespace height_mapping
