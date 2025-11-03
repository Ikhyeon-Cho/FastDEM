/*
 * height_mapping_node.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <grid_map_msgs/GridMap.h>
#include <logger/logger.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <memory>

#include "height_mapping/mapper.h"
#include "height_mapping_ros/adapters/pointcloud_converter.h"
#include "height_mapping_ros/adapters/tf2_transform.h"
#include "height_mapping_ros/node_parameters.h"

namespace height_mapping::ros {

static constexpr const char *LABEL = "height_mapping_node";

class MappingNode {
 public:
  MappingNode()
      : pnh_{"~"},
        cfg_{pnh_},
        tf_tree_{createTFTree(cfg_)},
        mapper_{createMapper(tf_tree_, cfg_)} {
    setupROSInterfaces();
    setupBenchmark();
    printNodeInfo();
  }

 private:
  TF2Lookup::Ptr createTFTree(const NodeParameters &cfg) {
    auto tf_tree = std::make_shared<TF2Lookup>();
    tf_tree->setLookupTimeout(cfg.tf_tree.lookup_timeout);
    tf_tree->setMaxExtrapolationTime(cfg.tf_tree.max_extrapolation_time);
    return tf_tree;
  }

  HeightMapper::Ptr createMapper(TF2Lookup::Ptr tf_tree,
                                 const NodeParameters &cfg) {
    try {
      return std::make_unique<HeightMapper>(
          tf_tree, Config::fromFile(cfg.mapper.config_file));
    } catch (const std::exception &e) {
      LOG_ERROR(LABEL, "Failed to create Height mapper: ", e.what());
      throw;
    }
  }

  void setupROSInterfaces() {
    sub_scan_ = pnh_.subscribe(cfg_.topics.scan, 10,
                               &MappingNode::pointcloudCallback, this);
    pub_scan_processed_ =
        pnh_.advertise<sensor_msgs::PointCloud2>(cfg_.topics.scan_processed, 1);
    pub_heightmap_ =
        pnh_.advertise<grid_map_msgs::GridMap>(cfg_.topics.heightmap, 1);

    if (cfg_.publish_rate > 0.0)
      pub_timer_ = pnh_.createTimer(::ros::Duration(1.0 / cfg_.publish_rate),
                                    &MappingNode::publishTimerCallback, this);
  }

  void printNodeInfo() const {
    // Set logger level
    logger::setLevelFromString(cfg_.logger_level);

    if (tf_tree_) LOG_INFO(LABEL, "Tf2 Transform ready [OK]");
    if (mapper_) LOG_INFO(LABEL, "Mapping engine ready [OK]");
    LOG_INFO(LABEL, "ROS interfaces ready [OK]");

    // Extract filename from path
    size_t last_slash = cfg_.mapper.config_file.find_last_of("/");
    std::string cfg_filename =
        (last_slash != std::string::npos)
            ? cfg_.mapper.config_file.substr(last_slash + 1)
            : cfg_.mapper.config_file;

    LOG_NOTICE(" ");
    LOG_NOTICE_BOLD("===== Configuration Summary =====");
    LOG_NOTICE("Config: ", cfg_filename);
    LOG_NOTICE("Input:  ", sub_scan_.getTopic());
    LOG_NOTICE("Output: ", pub_heightmap_.getTopic());
    LOG_NOTICE("Publish rate: ", cfg_.publish_rate, " Hz");
    LOG_NOTICE("Benchmarking: ",
               cfg_.mapper.benchmark.enabled ? "Enabled" : "Disabled");
    LOG_NOTICE("Logger level: ", cfg_.logger_level);
    LOG_NOTICE_BOLD("=================================");
    LOG_NOTICE(" ");
    LOG_INFO(LABEL, "Height Mapping Node successfully initialized!");
  }

  void setupBenchmark() {
    mapper_->setBenchmarkEnabled(cfg_.mapper.benchmark.enabled);
    mapper_->setBenchmarkInterval(cfg_.mapper.benchmark.report_interval);
    if (mapper_->getProfiler()) {
      mapper_->getProfiler()->setLogEachStage(
          cfg_.mapper.benchmark.log_each_stage);
    }
  }

  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    try {
      mapper_->integrate(adapters::fromPointCloud2(*msg));

      // Publish processed scan if available
      auto processed = mapper_->getScanProcessed();
      if (processed) {
        publishScanProcessed(adapters::toPointCloud2(*processed));
      }
    } catch (const std::exception &e) {
      LOG_ERROR_THROTTLE(1.0, LABEL,
                         "Error processing point cloud: ", e.what());
    }
  }

  void publishScanProcessed(const sensor_msgs::PointCloud2 &msg) {
    if (pub_scan_processed_.getNumSubscribers() == 0) return;
    pub_scan_processed_.publish(msg);
  }

  void publishHeightMap(const grid_map_msgs::GridMap &msg) {
    if (pub_heightmap_.getNumSubscribers() == 0) return;
    pub_heightmap_.publish(msg);
  }

  void publishTimerCallback(const ::ros::TimerEvent &event) {
    auto height_map = mapper_->getHeightMap();
    if (height_map) {
      grid_map_msgs::GridMap msg;
      grid_map::GridMapRosConverter::toMessage(height_map->toGridMap(), msg);
      publishHeightMap(msg);
    }
  }

  ::ros::NodeHandle pnh_{"~"};
  NodeParameters cfg_{pnh_};

  TF2Lookup::Ptr tf_tree_;
  HeightMapper::Ptr mapper_;

  ::ros::Subscriber sub_scan_;
  ::ros::Publisher pub_scan_processed_;
  ::ros::Publisher pub_heightmap_;
  ::ros::Timer pub_timer_;
};

}  // namespace height_mapping::ros

int main(int argc, char **argv) {
  using namespace height_mapping::ros;

  ros::init(argc, argv, LABEL);
  try {
    MappingNode node;
    ros::spin();
  } catch (const std::exception &e) {
    LOG_ERROR("main", "Fatal error: ", e.what());
    return 1;
  }

  return 0;
}