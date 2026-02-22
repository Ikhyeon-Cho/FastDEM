// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <spdlog/spdlog.h>
#include <visualization_msgs/msg/marker.hpp>

#include <fastdem/fastdem.hpp>
#include <fastdem/postprocess/feature_extraction.hpp>
#include <fastdem/postprocess/inpainting.hpp>
#include <fastdem/postprocess/uncertainty_fusion.hpp>
#include <memory>
#include <nanopcl/bridge/ros2.hpp>
#include <shared_mutex>

#include "fastdem_ros/conversions.hpp"
#include "fastdem_ros/parameters.hpp"
#include "fastdem_ros/tf_bridge.hpp"

namespace fastdem::ros2 {

using fastdem::ElevationMap;
using fastdem::FastDEM;
using fastdem::PointCloud;

class MappingNode : public rclcpp::Node {
 public:
  MappingNode() : rclcpp::Node("fastdem") {}

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
    this->declare_parameter<std::string>("config_file", "");
    std::string config_path =
        this->get_parameter("config_file").as_string();

    // Launch arg override: input_scan
    this->declare_parameter<std::string>("input_scan", "");

    try {
      cfg_ = NodeConfig::load(config_path);
    } catch (const std::exception& e) {
      spdlog::error("Failed to load config: {}", e.what());
      return false;
    }

    auto input_scan = this->get_parameter("input_scan").as_string();
    if (!input_scan.empty()) {
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

    // ROS TF provides both Calibration and Odometry
    auto tf = std::make_shared<TFBridge>(
        this->get_clock(),         //
        cfg_.tf.base_frame,        //
        cfg_.tf.map_frame,         //
        cfg_.tf.max_wait_time,     //
        cfg_.tf.max_stale_time);   //
    mapper_->setTransformProvider(tf);

    // Register callbacks to publish intermediate clouds
    mapper_->onPreprocessed(
        [this](const PointCloud& cloud) { publishProcessedScan(cloud); });
    mapper_->onRasterized(
        [this](const PointCloud& cloud) { publishRasterizedScan(cloud); });
  }

  void setupPublishersAndSubscribers() {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cfg_.topics.input_scan, rclcpp::SensorDataQoS(),
        std::bind(&MappingNode::scanCallback, this, std::placeholders::_1));

    pub_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/mapping/scan", 1);
    pub_rasterized_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/mapping/rasterized", 1);
    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/mapping/map", 1);
    pub_gridmap_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "~/mapping/gridmap", 1);
    pub_boundary_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "~/mapping/boundary", 1);

    pub_post_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/postprocess/map", 1);
    pub_post_gridmap_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "~/postprocess/gridmap", 1);

    auto period = std::chrono::duration<double>(1.0 / cfg_.topics.publish_rate);
    timer_viz_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&MappingNode::publishMap, this));
  }

  // ==================== Callbacks ====================

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    static bool first_scan = true;
    if (first_scan) {
      spdlog::info("First scan received. Mapping started...");
      if (cfg_.postprocess.uncertainty_fusion.enabled ||
          cfg_.postprocess.inpainting.enabled ||
          cfg_.postprocess.feature_extraction.enabled) {
        auto period = std::chrono::duration<double>(
            1.0 / cfg_.topics.post_process_rate);
        timer_post_process_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&MappingNode::postProcessCallback, this));
      }
      first_scan = false;
    }

    auto cloud = std::make_shared<PointCloud>(nanopcl::from(*msg));
    std::unique_lock lock(map_mutex_);
    mapper_->integrate(cloud);
  }

  void postProcessCallback() {
    ElevationMap map_raw;
    {
      std::shared_lock lock(map_mutex_);
      if (!map_.exists(layer::elevation)) return;
      map_raw = map_.snapshot(
          {layer::elevation, layer::upper_bound, layer::lower_bound});
    }

    // Post-processing on snapshot (lock-free)
    const auto& pp = cfg_.postprocess;
    if (pp.uncertainty_fusion.enabled)
      applyUncertaintyFusion(map_raw, pp.uncertainty_fusion);
    if (pp.inpainting.enabled)
      applyInpainting(map_raw, pp.inpainting.max_iterations,
                      pp.inpainting.min_valid_neighbors, /*inplace=*/true);
    if (pp.feature_extraction.enabled)
      applyFeatureExtraction(map_raw, pp.feature_extraction.analysis_radius,
                             pp.feature_extraction.min_valid_neighbors);

    // Compute derived layer for visualization
    grid_map::Matrix range_mat =
        map_raw.get(layer::upper_bound) - map_raw.get(layer::lower_bound);
    map_raw.add("uncertainty_range", range_mat);

    // Publish
    if (pub_post_map_->get_subscription_count() > 0)
      pub_post_map_->publish(ros2::toPointCloud2(map_raw));
    if (pub_post_gridmap_->get_subscription_count() > 0 &&
        cfg_.pipeline.mapping.mode != fastdem::MappingMode::GLOBAL) {
      grid_map_msgs::msg::GridMap gridmap_msg;
      toMessage(map_raw, gridmap_msg);
      pub_post_gridmap_->publish(gridmap_msg);
    }
  }

  // ==================== Publishers ====================

  void publishMap() {
    const bool want_map = pub_map_->get_subscription_count() > 0;
    const bool want_gridmap =
        pub_gridmap_->get_subscription_count() > 0 &&
        cfg_.pipeline.mapping.mode != fastdem::MappingMode::GLOBAL;
    const bool want_boundary = pub_boundary_->get_subscription_count() > 0;
    if (want_map || want_gridmap || want_boundary) {
      std::shared_lock lock(map_mutex_);
      if (want_map) pub_map_->publish(ros2::toPointCloud2(map_));
      if (want_gridmap) {
        grid_map_msgs::msg::GridMap msg;
        toMessage(map_, msg);
        pub_gridmap_->publish(msg);
      }
      if (want_boundary) pub_boundary_->publish(ros2::toRegionMarker(map_));
    }
  }

  void publishProcessedScan(const PointCloud& cloud) {
    if (pub_scan_->get_subscription_count() == 0) return;
    pub_scan_->publish(nanopcl::to(cloud));
  }

  void publishRasterizedScan(const PointCloud& cloud) {
    if (pub_rasterized_->get_subscription_count() == 0) return;
    pub_rasterized_->publish(nanopcl::to(cloud));
  }

  void printNodeSummary() const {
    spdlog::info("");
    spdlog::info("===== FastDEM Mapping Node =====");
    spdlog::info("  Input    : {}", cfg_.topics.input_scan);
    spdlog::info("  Output   : {}/mapping/map", this->get_name());
    spdlog::info("  Pub rate : {} Hz", cfg_.topics.publish_rate);
    spdlog::info("================================");
    spdlog::info("");
  }

  NodeConfig cfg_;

  // Core objects
  ElevationMap map_;
  std::unique_ptr<FastDEM> mapper_;
  mutable std::shared_mutex map_mutex_;

  // ROS2 handles
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_rasterized_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_gridmap_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_boundary_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_post_map_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_post_gridmap_;
  rclcpp::TimerBase::SharedPtr timer_viz_;
  rclcpp::TimerBase::SharedPtr timer_post_process_;
};

}  // namespace fastdem::ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fastdem::ros2::MappingNode>();
  if (!node->initialize()) {
    return 1;
  }

  // Multi-threaded executor for parallel processing:
  // Thread 1: Point cloud processing (mapping)
  // Thread 2: Async post-processing (inpainting, feature extraction)
  // Thread 3: Map publishing (visualization)
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
