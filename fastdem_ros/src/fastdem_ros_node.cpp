// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <spdlog/spdlog.h>
#include <visualization_msgs/Marker.h>

#include <fastdem/fastdem.hpp>
#include <fastdem/postprocess/feature_extraction.hpp>
#include <fastdem/postprocess/inpainting.hpp>
#include <fastdem/postprocess/uncertainty_fusion.hpp>
#include <memory>
#include <shared_mutex>

#include "fastdem_ros/conversions.hpp"
#include "fastdem_ros/parameters.hpp"
#include "fastdem_ros/tf_bridge.hpp"

namespace fastdem::ros1 {

using fastdem::ElevationMap;
using fastdem::FastDEM;
using fastdem::PointCloud;

class MappingNode {
 public:
  MappingNode() : nh_{"~"} {}

  bool initialize() {
    if (!loadParameters()) return false;

    setupCore();
    setupPublishersAndSubscribers();
    printNodeSummary();

    spdlog::info("FastDEM node initialized successfully!");
    return true;
  }

 private:
  bool loadParameters() {
    std::string config_path;
    nh_.param<std::string>("config_file", config_path, std::string{});
    try {
      cfg_ = NodeConfig::load(config_path);
    } catch (const std::exception& e) {
      spdlog::error("Failed to load config: {}", e.what());
      return false;
    }

    // Launch arg override: input_scan
    std::string input_scan;
    if (nh_.getParam("input_scan", input_scan)) {
      cfg_.topics.input_scan = input_scan;
    }

    spdlog::set_level(spdlog::level::from_str(cfg_.logger_level));
    return true;
  }

  void setupCore() {
    // Elevation map geometry
    map_.setGeometry(cfg_.map.width, cfg_.map.height, cfg_.map.resolution);
    map_.setFrameId(cfg_.tf.map_frame);

    // Mapper
    mapper_ = std::make_unique<FastDEM>(map_, cfg_.pipeline);
    mapper_->setMappingMode(cfg_.mapping_mode);

    // Set scan data limit
    mapper_->setHeightFilter(cfg_.point_filter.z_min, cfg_.point_filter.z_max);
    mapper_->setRangeFilter(cfg_.point_filter.range_min,
                            cfg_.point_filter.range_max);

    // ROS TF provides both Calibration and Odometry values
    auto tf = std::make_shared<TFBridge>(cfg_.tf.base_frame, cfg_.tf.map_frame);
    tf->setMaxWaitTime(cfg_.tf.max_wait_time);
    tf->setMaxStaleTime(cfg_.tf.max_stale_time);
    mapper_->setTransformSystem(tf);

    // Register callback to publish processed cloud
    mapper_->setProcessedCloudCallback(
        [this](const PointCloud& cloud) { publishProcessedScan(cloud); });
  }

  void setupPublishersAndSubscribers() {
    // clang-format off
    sub_scan_ = nh_.subscribe(cfg_.topics.input_scan, 10, &MappingNode::scanCallback, this);
    
    pub_scan_         = nh_.advertise<sensor_msgs::PointCloud2>("mapping/scan", 1);
    pub_map_          = nh_.advertise<sensor_msgs::PointCloud2>("mapping/map", 1);
    pub_gridmap_      = nh_.advertise<grid_map_msgs::GridMap>("mapping/gridmap", 1);
    pub_boundary_     = nh_.advertise<visualization_msgs::Marker>("mapping/boundary", 1);
    
    pub_post_map_     = nh_.advertise<sensor_msgs::PointCloud2>("postprocess/map", 1);
    pub_post_gridmap_ = nh_.advertise<grid_map_msgs::GridMap>("postprocess/gridmap", 1);

    timer_viz_ = nh_.createTimer(ros::Duration(1.0 / cfg_.topics.publish_rate),
                                     &MappingNode::publishMap, this);
    // clang-format on
  }

  // ==================== Callbacks ====================

  void scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    static bool first_scan = true;
    if (first_scan) {
      spdlog::info("First scan received. Mapping started...");
      if (cfg_.pipeline.uncertainty_fusion.enabled || cfg_.inpainting.enabled ||
          cfg_.feature_extraction.enabled)
        timer_post_process_ =
            nh_.createTimer(ros::Duration(1.0 / cfg_.topics.post_process_rate),
                            &MappingNode::postProcessCallback, this);
      first_scan = false;
    }

    auto cloud = std::make_shared<PointCloud>(nanopcl::from(*msg));
    std::unique_lock lock(map_mutex_);
    mapper_->integrate(cloud);
  }

  void postProcessCallback(const ros::TimerEvent&) {
    ElevationMap map_raw;
    {
      std::shared_lock lock(map_mutex_);
      if (!map_.exists(layer::elevation)) return;
      map_raw =
          map_.snapshot({layer::elevation, layer::state, layer::variance});
    }

    // Post-processing on snapshot (lock-free)
    if (cfg_.pipeline.uncertainty_fusion.enabled)
      applyUncertaintyFusion(map_raw, cfg_.pipeline.uncertainty_fusion);
    if (cfg_.inpainting.enabled)
      applyInpainting(map_raw, cfg_.inpainting.max_iterations,
                      cfg_.inpainting.min_valid_neighbors, /*inplace=*/true);
    if (cfg_.feature_extraction.enabled)
      applyFeatureExtraction(map_raw,
                             cfg_.feature_extraction.analysis_radius,  //
                             cfg_.feature_extraction.min_valid_neighbors);

    // Publish
    if (pub_post_map_.getNumSubscribers() > 0)
      pub_post_map_.publish(ros1::toPointCloud2(map_raw));
    if (pub_post_gridmap_.getNumSubscribers() > 0 &&
        cfg_.mapping_mode != fastdem::MappingMode::GLOBAL)
      pub_post_gridmap_.publish(ros1::toGridMap(map_raw));
  }

  // ==================== Publishers ====================

  void publishMap(const ros::TimerEvent&) {
    // Core layers (from map_)
    const bool want_map = pub_map_.getNumSubscribers() > 0;
    const bool want_gridmap = pub_gridmap_.getNumSubscribers() > 0 &&
                              cfg_.mapping_mode != fastdem::MappingMode::GLOBAL;
    const bool want_boundary = pub_boundary_.getNumSubscribers() > 0;
    if (want_map || want_gridmap || want_boundary) {
      std::shared_lock lock(map_mutex_);
      if (want_map) pub_map_.publish(ros1::toPointCloud2(map_));
      if (want_gridmap) pub_gridmap_.publish(ros1::toGridMap(map_));
      if (want_boundary) pub_boundary_.publish(ros1::toMapBoundary(map_));
    }
  }

  void publishProcessedScan(const PointCloud& cloud) {
    if (pub_scan_.getNumSubscribers() == 0) return;
    pub_scan_.publish(nanopcl::to(cloud));
  }

  void printNodeSummary() const {
    spdlog::info("");
    spdlog::info("===== FastDEM Mapping Node =====");
    spdlog::info("  Input    : {}", sub_scan_.getTopic());
    spdlog::info("  Output   : {}", pub_map_.getTopic());
    spdlog::info("  Pub rate : {} Hz", cfg_.topics.publish_rate);
    spdlog::info("================================");
    spdlog::info("");
  }

  ros::NodeHandle nh_;
  NodeConfig cfg_;

  // Core objects
  ElevationMap map_;
  std::unique_ptr<FastDEM> mapper_;
  mutable std::shared_mutex map_mutex_;

  // ROS handles
  ros::Subscriber sub_scan_;
  ros::Publisher pub_scan_;
  ros::Publisher pub_map_;
  ros::Publisher pub_gridmap_;
  ros::Publisher pub_boundary_;
  ros::Publisher pub_post_map_;
  ros::Publisher pub_post_gridmap_;
  ros::Timer timer_viz_;
  ros::Timer timer_post_process_;
};

}  // namespace fastdem::ros1

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastdem_ros_node");
  fastdem::ros1::MappingNode node;
  if (!node.initialize()) {
    return 1;
  }

  // Multi-threaded spinner for parallel processing:
  // Thread 1: Point cloud processing (mapping)
  // Thread 2: Async post-processing (inpainting, feature extraction)
  // Thread 3: Map publishing (visualization)
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
