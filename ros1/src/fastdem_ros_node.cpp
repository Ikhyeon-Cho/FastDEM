// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <spdlog/spdlog.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <fastdem/bridge/ros1.hpp>
#include <fastdem/fastdem.hpp>
#include <fastdem/postprocess/feature_extraction.hpp>
#include <fastdem/postprocess/inpainting.hpp>
#include <fastdem/postprocess/uncertainty_fusion.hpp>
#include <memory>
#include <nanopcl/bridge/ros1.hpp>
#include <shared_mutex>

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

    // Launch arg override: single input_scan → 1-element list
    std::string input_scan;
    if (nh_.getParam("input_scan", input_scan)) {
      cfg_.topics.input_scans = {input_scan};
    }

    spdlog::set_level(spdlog::level::from_str(cfg_.logger_level));
    return true;
  }

  void setupCore() {
    // Elevation map
    map_.setGeometry(cfg_.map.width, cfg_.map.height, cfg_.map.resolution);
    map_.setFrameId(cfg_.tf.map_frame);

    // Mapper
    mapper_ = std::make_unique<FastDEM>(map_, cfg_.pipeline);

    // ROS TF provides both Calibration and Odometry values
    tf_ = std::make_shared<TFBridge>(cfg_.tf.base_frame, cfg_.tf.map_frame);
    tf_->setMaxWaitTime(cfg_.tf.max_wait_time);
    tf_->setMaxStaleTime(cfg_.tf.max_stale_time);
    mapper_->setTransformProvider(tf_);

    // Register callbacks to publish intermediate clouds
    mapper_->onScanPreprocessed(
        [this](const PointCloud& cloud) { publishProcessedScan(cloud); });
    mapper_->onScanRasterized(
        [this](const PointCloud& cloud) { publishRasterizedScan(cloud); });
  }

  void setupPublishersAndSubscribers() {
    // clang-format off
    for (const auto& topic : cfg_.topics.input_scans)
      scan_subs_.push_back(nh_.subscribe(topic, 10, &MappingNode::scanCallback, this));
    
    pub_scan_         = nh_.advertise<sensor_msgs::PointCloud2>("scan/preprocessed", 1);
    pub_rasterized_   = nh_.advertise<sensor_msgs::PointCloud2>("scan/rasterized", 1);
    pub_map_          = nh_.advertise<sensor_msgs::PointCloud2>("mapping/cloud", 1);
    pub_gridmap_      = nh_.advertise<grid_map_msgs::GridMap>("mapping/gridmap", 1);
    pub_boundary_     = nh_.advertise<visualization_msgs::Marker>("mapping/boundary", 1);
    pub_global_map_   = nh_.advertise<sensor_msgs::PointCloud2>("mapping/cloud_global", 1);

    pub_post_map_     = nh_.advertise<sensor_msgs::PointCloud2>("postprocess/cloud", 1);
    pub_post_gridmap_ = nh_.advertise<grid_map_msgs::GridMap>("postprocess/gridmap", 1);
    pub_normals_      = nh_.advertise<visualization_msgs::MarkerArray>("postprocess/normals", 1);

    timer_viz_ = nh_.createTimer(ros::Duration(1.0 / cfg_.topics.publish_rate),
                                     &MappingNode::publishLocalView, this);
    if (cfg_.pipeline.mapping.mode == MappingMode::GLOBAL)
      timer_global_ = nh_.createTimer(ros::Duration(1.0 / cfg_.topics.global_publish_rate),
                                          &MappingNode::publishGlobalView, this);

    const bool has_postproc = cfg_.postprocess.uncertainty_fusion.enabled ||
                              cfg_.postprocess.inpainting.enabled ||
                              cfg_.postprocess.feature_extraction.enabled;
    if (has_postproc && cfg_.topics.post_process_rate > 0.0)
      timer_post_process_ =
          nh_.createTimer(ros::Duration(1.0 / cfg_.topics.post_process_rate),
                          &MappingNode::postProcessCallback, this,
                          /*oneshot=*/false, /*autostart=*/false);

    srv_reset_       = nh_.advertiseService("reset_map", &MappingNode::resetMapCallback, this);
    srv_postprocess_ = nh_.advertiseService("run_postprocess", &MappingNode::runPostProcessCallback, this);
    srv_inpainting_  = nh_.advertiseService("run_inpainting", &MappingNode::runInpaintingCallback, this);
    srv_uf_          = nh_.advertiseService("run_uncertainty_fusion", &MappingNode::runUncertaintyFusionCallback, this);
    srv_fe_          = nh_.advertiseService("run_feature_extraction", &MappingNode::runFeatureExtractionCallback, this);
    // clang-format on
  }

  // ==================== Services ====================

  bool resetMapCallback(std_srvs::Trigger::Request&,
                        std_srvs::Trigger::Response& res) {
    std::unique_lock lock(map_mutex_);
    mapper_->reset();
    res.success = true;
    res.message = "Map reset";
    spdlog::info("Map reset by service call");
    return true;
  }

  bool runPostProcessCallback(std_srvs::Trigger::Request&,
                              std_srvs::Trigger::Response& res) {
    runPostProcess(true, true, true);
    res.success = true;
    res.message = "Post-processing complete";
    return true;
  }

  bool runInpaintingCallback(std_srvs::Trigger::Request&,
                             std_srvs::Trigger::Response& res) {
    runPostProcess(false, true, false);
    res.success = true;
    res.message = "Inpainting complete";
    return true;
  }

  bool runUncertaintyFusionCallback(std_srvs::Trigger::Request&,
                                    std_srvs::Trigger::Response& res) {
    runPostProcess(true, false, false);
    res.success = true;
    res.message = "Uncertainty fusion complete";
    return true;
  }

  bool runFeatureExtractionCallback(std_srvs::Trigger::Request&,
                                    std_srvs::Trigger::Response& res) {
    runPostProcess(false, false, true);
    res.success = true;
    res.message = "Feature extraction complete";
    return true;
  }

  // ==================== Process ====================

  void scanCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    static bool first_scan = true;
    if (first_scan) {
      spdlog::info("First scan received. Mapping started...");
      if (timer_post_process_) timer_post_process_.start();
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

  void postProcessCallback(const ros::TimerEvent&) {
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
    if (pub_post_map_.getNumSubscribers() > 0)
      pub_post_map_.publish(ros1::toPointCloud2(map_copy));
    if (pub_post_gridmap_.getNumSubscribers() > 0 &&
        cfg_.pipeline.mapping.mode != fastdem::MappingMode::GLOBAL)
      pub_post_gridmap_.publish(ros1::toGridMap(map_copy));
    if (pub_normals_.getNumSubscribers() > 0 && do_fe) {
      const auto& nm = cfg_.visualization.feature_extraction.normals;
      pub_normals_.publish(
          ros1::toNormalMarkers(map_copy, nm.arrow_length, nm.stride));
    }
  }

  // ==================== Publishers ====================

  void publishLocalView(const ros::TimerEvent&) {
    const bool is_global = cfg_.pipeline.mapping.mode == MappingMode::GLOBAL;
    const bool want_cloud = pub_map_.getNumSubscribers() > 0;
    const bool want_gridmap =
        pub_gridmap_.getNumSubscribers() > 0 && !is_global;
    const bool want_boundary = pub_boundary_.getNumSubscribers() > 0;
    if (!want_cloud && !want_gridmap && !want_boundary) return;

    std::shared_lock lock(map_mutex_);
    if (want_cloud) publishMapCloud();
    if (want_gridmap) publishGridMap();
    if (want_boundary) publishMapBoundary();
  }

  void publishGlobalView(const ros::TimerEvent&) {
    if (pub_global_map_.getNumSubscribers() == 0) return;
    std::shared_lock lock(map_mutex_);
    pub_global_map_.publish(ros1::toPointCloud2(map_));
  }

  // --- internal publish helpers (caller must hold map_mutex_) ---

  void publishMapCloud() {
    if (cfg_.pipeline.mapping.mode == MappingMode::GLOBAL) {
      auto pose = tf_->getPoseAt(map_.getTimestamp());
      if (pose)
        pub_map_.publish(ros1::toPointCloud2(
            map_, pose->translation().head<2>(), {15.0, 15.0}));
    } else {
      pub_map_.publish(ros1::toPointCloud2(map_));
    }
  }

  void publishGridMap() { pub_gridmap_.publish(ros1::toGridMap(map_)); }

  void publishMapBoundary() {
    pub_boundary_.publish(ros1::toMapBoundary(map_));
  }

  void publishProcessedScan(const PointCloud& cloud) {
    if (pub_scan_.getNumSubscribers() == 0) return;
    pub_scan_.publish(nanopcl::to(cloud));
  }

  void publishRasterizedScan(const PointCloud& cloud) {
    if (pub_rasterized_.getNumSubscribers() == 0) return;
    pub_rasterized_.publish(nanopcl::to(cloud));
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
    if (scan_subs_.size() == 1) {
      spdlog::info("  Input      : {}", scan_subs_[0].getTopic());
    } else {
      for (size_t i = 0; i < scan_subs_.size(); ++i)
        spdlog::info("  Input [{}]  : {}", i, scan_subs_[i].getTopic());
    }
    if (postproc != "none")
      spdlog::info("  Pub rate   : {} Hz (post: {} Hz)",
                   cfg_.topics.publish_rate, cfg_.topics.post_process_rate);
    else
      spdlog::info("  Pub rate   : {} Hz", cfg_.topics.publish_rate);
    spdlog::info("================================");
    spdlog::info("");
  }

  ros::NodeHandle nh_;
  NodeConfig cfg_;

  // Core objects
  ElevationMap map_;
  std::unique_ptr<FastDEM> mapper_;
  std::shared_ptr<TFBridge> tf_;
  mutable std::shared_mutex map_mutex_;
  bm::Profiler pp_profiler_;  // Post-processing profiler

  // ROS handles
  std::vector<ros::Subscriber> scan_subs_;
  ros::Publisher pub_scan_;
  ros::Publisher pub_rasterized_;
  ros::Publisher pub_map_;
  ros::Publisher pub_gridmap_;
  ros::Publisher pub_boundary_;
  ros::Publisher pub_global_map_;
  ros::Publisher pub_post_map_;
  ros::Publisher pub_post_gridmap_;
  ros::Publisher pub_normals_;
  ros::ServiceServer srv_reset_;
  ros::ServiceServer srv_postprocess_;
  ros::ServiceServer srv_inpainting_;
  ros::ServiceServer srv_uf_;
  ros::ServiceServer srv_fe_;
  ros::Timer timer_viz_;
  ros::Timer timer_global_;
  ros::Timer timer_post_process_;
};

}  // namespace fastdem::ros1

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastdem_node");
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
