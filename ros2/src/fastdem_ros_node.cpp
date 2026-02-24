// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <spdlog/spdlog.h>

#include <fastdem/bridge/ros2.hpp>
#include <fastdem/fastdem.hpp>
#include <fastdem/postprocess/feature_extraction.hpp>
#include <fastdem/postprocess/inpainting.hpp>
#include <fastdem/postprocess/uncertainty_fusion.hpp>
#include <memory>
#include <nanopcl/bridge/ros2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <shared_mutex>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "fastdem_ros/parameters.hpp"
#include "fastdem_ros/tf_bridge.hpp"

namespace fastdem::ros2 {

using fastdem::ElevationMap;
using fastdem::FastDEM;
using fastdem::PointCloud;

class MappingNode : public rclcpp::Node {
 public:
  MappingNode() : rclcpp::Node("fastdem_node") {}

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
    std::string config_path = this->get_parameter("config_file").as_string();

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
      cfg_.topics.input_scans = {input_scan};
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
    tf_ = std::make_shared<TFBridge>(this->get_clock(),        //
                                     cfg_.tf.base_frame,       //
                                     cfg_.tf.map_frame,        //
                                     cfg_.tf.max_wait_time,    //
                                     cfg_.tf.max_stale_time);  //
    mapper_->setTransformProvider(tf_);

    // Register callbacks to publish intermediate clouds
    mapper_->onScanPreprocessed(
        [this](const PointCloud& cloud) { publishProcessedScan(cloud); });
    mapper_->onScanRasterized(
        [this](const PointCloud& cloud) { publishRasterizedScan(cloud); });
  }

  void setupPublishersAndSubscribers() {
    // clang-format off
    using Cloud       = sensor_msgs::msg::PointCloud2;
    using GridMapMsg  = grid_map_msgs::msg::GridMap;
    using Marker      = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Trigger     = std_srvs::srv::Trigger;

    for (const auto& topic : cfg_.topics.input_scans)
      scan_subs_.push_back(this->create_subscription<Cloud>(
          topic, rclcpp::SensorDataQoS(),
          std::bind(&MappingNode::scanCallback, this, std::placeholders::_1)));

    pub_scan_         = this->create_publisher<Cloud>("~/scan/preprocessed", 1);
    pub_rasterized_   = this->create_publisher<Cloud>("~/scan/rasterized", 1);
    pub_map_          = this->create_publisher<Cloud>("~/mapping/cloud", 1);
    pub_gridmap_      = this->create_publisher<GridMapMsg>("~/mapping/gridmap", 1);
    pub_boundary_     = this->create_publisher<Marker>("~/mapping/boundary", 1);
    pub_global_map_   = this->create_publisher<Cloud>("~/mapping/cloud_global", 1);

    pub_post_map_     = this->create_publisher<Cloud>("~/postprocess/cloud", 1);
    pub_post_gridmap_ = this->create_publisher<GridMapMsg>("~/postprocess/gridmap", 1);
    pub_normals_      = this->create_publisher<MarkerArray>("~/postprocess/normals", 1);

    auto to_ms = [](double rate) {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::duration<double>(1.0 / rate));
    };

    timer_viz_ = this->create_wall_timer(
        to_ms(cfg_.topics.publish_rate),
        std::bind(&MappingNode::publishLocalView, this));
    if (cfg_.pipeline.mapping.mode == MappingMode::GLOBAL)
      timer_global_ = this->create_wall_timer(
          to_ms(cfg_.topics.global_publish_rate),
          std::bind(&MappingNode::publishGlobalView, this));

    auto bind_srv = [this](auto callback) {
      return std::bind(callback, this, std::placeholders::_1, std::placeholders::_2);
    };
    srv_reset_       = this->create_service<Trigger>("~/reset_map",             bind_srv(&MappingNode::resetMapCallback));
    srv_postprocess_ = this->create_service<Trigger>("~/run_postprocess",       bind_srv(&MappingNode::runPostProcessCallback));
    srv_inpainting_  = this->create_service<Trigger>("~/run_inpainting",        bind_srv(&MappingNode::runInpaintingCallback));
    srv_uf_          = this->create_service<Trigger>("~/run_uncertainty_fusion", bind_srv(&MappingNode::runUncertaintyFusionCallback));
    srv_fe_          = this->create_service<Trigger>("~/run_feature_extraction", bind_srv(&MappingNode::runFeatureExtractionCallback));
    // clang-format on
  }

  // ==================== Services ====================

  using TriggerRequest = std::shared_ptr<std_srvs::srv::Trigger::Request>;
  using TriggerResponse = std::shared_ptr<std_srvs::srv::Trigger::Response>;

  void resetMapCallback(const TriggerRequest, TriggerResponse res) {
    std::unique_lock lock(map_mutex_);
    mapper_->reset();
    res->success = true;
    res->message = "Map reset";
    spdlog::info("Map reset by service call");
  }

  void runPostProcessCallback(const TriggerRequest, TriggerResponse res) {
    runPostProcess(true, true, true);
    res->success = true;
    res->message = "Post-processing complete";
  }

  void runInpaintingCallback(const TriggerRequest, TriggerResponse res) {
    runPostProcess(false, true, false);
    res->success = true;
    res->message = "Inpainting complete";
  }

  void runUncertaintyFusionCallback(const TriggerRequest, TriggerResponse res) {
    runPostProcess(true, false, false);
    res->success = true;
    res->message = "Uncertainty fusion complete";
  }

  void runFeatureExtractionCallback(const TriggerRequest, TriggerResponse res) {
    runPostProcess(false, false, true);
    res->success = true;
    res->message = "Feature extraction complete";
  }

  // ==================== Process ====================

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    static bool first_scan = true;
    if (first_scan) {
      spdlog::info("First scan received. Mapping started...");
      const bool has_postproc = cfg_.postprocess.uncertainty_fusion.enabled ||
                                cfg_.postprocess.inpainting.enabled ||
                                cfg_.postprocess.feature_extraction.enabled;
      if (has_postproc && cfg_.topics.post_process_rate > 0.0) {
        auto period =
            std::chrono::duration<double>(1.0 / cfg_.topics.post_process_rate);
        timer_post_process_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&MappingNode::postProcessCallback, this));
      }
      first_scan = false;
    }

    auto cloud = std::make_shared<PointCloud>(nanopcl::from(*msg));
    std::unique_lock lock(map_mutex_);
    mapper_->integrate(cloud);

    static size_t frame_count = 0;
    if (++frame_count % mapper_->profiler().windowSize() == 0) {
      mapper_->profiler().printTable();
    }
  }

  // ==================== Post-processing ====================

  void postProcessCallback() {
    const auto& pp = cfg_.postprocess;
    runPostProcess(pp.uncertainty_fusion.enabled, pp.inpainting.enabled,
                   pp.feature_extraction.enabled);

    static size_t pp_count = 0;
    if (++pp_count % pp_profiler_.windowSize() == 0) {
      pp_profiler_.printTable();
    }
  }

  void runPostProcess(bool do_uf, bool do_inpainting, bool do_fe) {
    auto _total = pp_profiler_.scope("Total");

    ElevationMap map_copy;
    std::shared_lock lock(map_mutex_, std::defer_lock);
    {
      auto _t = pp_profiler_.scope("Lock Wait");
      lock.lock();
    }
    {
      auto _t = pp_profiler_.scope("Snapshot");
      if (map_.isEmpty()) return;
      map_copy = map_.snapshot(
          {layer::elevation, layer::upper_bound, layer::lower_bound});
    }

    // Post-processing on snapshot (lock-free)
    const auto& pp = cfg_.postprocess;
    if (do_uf) {
      auto _t = pp_profiler_.scope("Uncertainty Fusion");
      applyUncertaintyFusion(map_copy, pp.uncertainty_fusion);
    }
    if (do_inpainting) {
      auto _t = pp_profiler_.scope("Inpainting");
      applyInpainting(map_copy, pp.inpainting.max_iterations,
                      pp.inpainting.min_valid_neighbors, /*inplace=*/true);
    }
    if (do_fe) {
      auto _t = pp_profiler_.scope("Feature Extraction");
      applyFeatureExtraction(map_copy, pp.feature_extraction.analysis_radius,
                             pp.feature_extraction.min_valid_neighbors,
                             pp.feature_extraction.step_lower_percentile,
                             pp.feature_extraction.step_upper_percentile);
    }

    // Compute derived layer for visualization
    grid_map::Matrix range_mat =
        map_copy.get(layer::upper_bound) - map_copy.get(layer::lower_bound);
    map_copy.add("uncertainty_range", range_mat);

    // Publish
    if (pub_post_map_->get_subscription_count() > 0)
      pub_post_map_->publish(ros2::toPointCloud2(map_copy));
    if (pub_post_gridmap_->get_subscription_count() > 0 &&
        cfg_.pipeline.mapping.mode != fastdem::MappingMode::GLOBAL)
      pub_post_gridmap_->publish(ros2::toGridMap(map_copy));
    if (pub_normals_->get_subscription_count() > 0 && do_fe) {
      const auto& nm = cfg_.visualization.feature_extraction.normals;
      pub_normals_->publish(
          ros2::toNormalMarkers(map_copy, nm.arrow_length, nm.stride));
    }
  }

  // ==================== Publishers ====================

  void publishLocalView() {
    const bool is_global = cfg_.pipeline.mapping.mode == MappingMode::GLOBAL;
    const bool want_cloud = pub_map_->get_subscription_count() > 0;
    const bool want_gridmap =
        pub_gridmap_->get_subscription_count() > 0 && !is_global;
    const bool want_boundary = pub_boundary_->get_subscription_count() > 0;
    if (!want_cloud && !want_gridmap && !want_boundary) return;

    std::shared_lock lock(map_mutex_);
    if (want_cloud) publishMapCloud();
    if (want_gridmap) publishGridMap();
    if (want_boundary) publishMapBoundary();
  }

  void publishGlobalView() {
    if (pub_global_map_->get_subscription_count() == 0) return;
    std::shared_lock lock(map_mutex_);
    pub_global_map_->publish(ros2::toPointCloud2(map_));
  }

  // --- internal publish helpers (caller must hold map_mutex_) ---

  void publishMapCloud() {
    if (cfg_.pipeline.mapping.mode == MappingMode::GLOBAL) {
      auto pose = tf_->getPoseAt(map_.getTimestamp());
      if (pose)
        pub_map_->publish(ros2::toPointCloud2(
            map_, pose->translation().head<2>(), {15.0, 15.0}));
    } else {
      pub_map_->publish(ros2::toPointCloud2(map_));
    }
  }

  void publishGridMap() { pub_gridmap_->publish(ros2::toGridMap(map_)); }

  void publishMapBoundary() {
    pub_boundary_->publish(ros2::toMapBoundary(map_));
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
    auto mode_str = [](MappingMode m) {
      return m == MappingMode::LOCAL ? "local" : "global";
    };
    auto est_str = [](EstimationType e) {
      return e == EstimationType::Kalman ? "kalman_filter" : "p2_quantile";
    };
    auto sensor_str = [](SensorType s) {
      switch (s) {
        case SensorType::Constant:
          return "constant";
        case SensorType::LiDAR:
          return "lidar";
        case SensorType::RGBD:
          return "rgbd";
        default:
          return "unknown";
      }
    };

    std::string postproc;
    const auto& pp = cfg_.postprocess;
    if (pp.uncertainty_fusion.enabled) postproc += "uncertainty_fusion, ";
    if (pp.inpainting.enabled) postproc += "inpainting, ";
    if (pp.feature_extraction.enabled) postproc += "feature_extraction, ";
    if (!postproc.empty())
      postproc.erase(postproc.size() - 2);
    else
      postproc = "none";

    const auto& m = cfg_.pipeline.mapping;

    spdlog::info("");
    spdlog::info("===== FastDEM Mapping Node =====");
    spdlog::info("  Map        : {} ({:.1f} x {:.1f} m, res={:.2f} m)",
                 mode_str(m.mode), cfg_.map.width, cfg_.map.height,
                 cfg_.map.resolution);
    spdlog::info("  Estimator  : {}", est_str(m.estimation_type));
    spdlog::info("  Sensor     : {}",
                 sensor_str(cfg_.pipeline.sensor_model.type));
    spdlog::info("  Raycasting : {}",
                 cfg_.pipeline.raycasting.enabled ? "on" : "off");
    spdlog::info("  Post-proc  : {}", postproc);
    spdlog::info("  TF         : {} -> {}", cfg_.tf.base_frame,
                 cfg_.tf.map_frame);
    if (cfg_.topics.input_scans.size() == 1) {
      spdlog::info("  Input      : {}", cfg_.topics.input_scans[0]);
    } else {
      for (size_t i = 0; i < cfg_.topics.input_scans.size(); ++i)
        spdlog::info("  Input [{}]  : {}", i, cfg_.topics.input_scans[i]);
    }
    if (postproc != "none")
      spdlog::info("  Pub rate   : {} Hz (post: {} Hz)",
                   cfg_.topics.publish_rate, cfg_.topics.post_process_rate);
    else
      spdlog::info("  Pub rate   : {} Hz", cfg_.topics.publish_rate);
    spdlog::info("================================");
    spdlog::info("");
  }

  NodeConfig cfg_;

  // Core objects
  ElevationMap map_;
  std::unique_ptr<FastDEM> mapper_;
  std::shared_ptr<TFBridge> tf_;
  mutable std::shared_mutex map_mutex_;
  bm::Profiler pp_profiler_;  // Post-processing profiler

  // clang-format off
  // ROS2 handles
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> scan_subs_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_rasterized_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_map_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr      pub_gridmap_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  pub_boundary_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_global_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_post_map_;
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr      pub_post_gridmap_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_normals_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_postprocess_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_inpainting_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_uf_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_fe_;
  rclcpp::TimerBase::SharedPtr timer_viz_;
  rclcpp::TimerBase::SharedPtr timer_global_;
  rclcpp::TimerBase::SharedPtr timer_post_process_;
  // clang-format on
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
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                    3);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
