/*
 * simple_mapper.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/simple_mapper.h"

#include "logger/logger.h"

namespace height_mapping {

constexpr const char *LABEL = "SimpleHeightMapper";

SimpleHeightMapper::SimpleHeightMapper(TransformLookup::Ptr tf, Config cfg)
    : cfg_(cfg), tf_tree_(tf), map_(initMap(cfg_)) {
  // Configure transformer to map frame
  algorithms::TransformCloud::Config transform_cfg;
  transform_cfg.target_frame = cfg_.map.frame_id;
  transformer_.setConfig(transform_cfg);

  // Configure estimator (incremental mean only for baseline)
  algorithms::HeightEstimation::Config estimator_cfg;
  estimator_cfg.estimator_type =
      algorithms::HeightEstimation::Config::EstimatorType::INCREMENTAL_MEAN;
  estimator_.setConfig(estimator_cfg);

  LOG_DEBUG(LABEL, "SimpleHeightMapper initialized");
}

HeightMap::Ptr SimpleHeightMapper::initMap(const Config &cfg) {
  auto map = std::make_shared<HeightMap>();
  map->configure(cfg.map.width, cfg.map.height, cfg.map.resolution);
  map->setFrameId(cfg.map.frame_id);
  return map;
}

void SimpleHeightMapper::integrate(std::shared_ptr<PointCloud> cloud) {
  // Validation
  if (!cloud || cloud->empty() || cloud->frameId().empty()) {
    LOG_ERROR(LABEL, "Invalid point cloud provided");
    return;
  }

  try {
    // 1. Transform to map frame
    if (cloud->frameId() != cfg_.map.frame_id) {
      auto transform = tf_tree_->lookupTransform(
          cfg_.map.frame_id, cloud->frameId(), cloud->timestamp());

      if (!transform) {
        LOG_WARN_THROTTLE(1.0, LABEL,
                          "Transform not available from ", cloud->frameId(),
                          " to ", cfg_.map.frame_id);
        return;
      }

      transformer_.transformInPlace(*cloud, *transform);
    }

    // 2. Update height map
    {
      if (cfg_.engine.thread_safe) {
        std::lock_guard<std::shared_mutex> lock(map_mutex_);
        estimator_.updateMap(*map_, *cloud);
      } else {
        estimator_.updateMap(*map_, *cloud);
      }
    }

    // 3. Store processed cloud
    {
      if (cfg_.engine.thread_safe) {
        std::lock_guard<std::shared_mutex> lock(cloud_mutex_);
        last_processed_cloud_ = cloud;
      } else {
        last_processed_cloud_ = cloud;
      }
    }

  } catch (const std::exception &e) {
    LOG_ERROR_THROTTLE(1.0, LABEL, "Processing failed: ", e.what());
    if (cfg_.engine.reset_on_error) {
      reset();
    }
  }
}

HeightMap::ConstPtr SimpleHeightMapper::getHeightMap() const {
  if (cfg_.engine.thread_safe) {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    return map_;
  }
  return map_;
}

std::shared_ptr<const PointCloud> SimpleHeightMapper::getScanProcessed() const {
  if (cfg_.engine.thread_safe) {
    std::shared_lock<std::shared_mutex> lock(cloud_mutex_);
    return last_processed_cloud_;
  }
  return last_processed_cloud_;
}

void SimpleHeightMapper::reset() {
  if (cfg_.engine.thread_safe) {
    std::lock_guard<std::shared_mutex> lock(map_mutex_);
    map_->clear();
  } else {
    map_->clear();
  }
}

}  // namespace height_mapping
