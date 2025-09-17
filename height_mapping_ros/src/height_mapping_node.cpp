/*
 * height_mapping_node.cpp
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/height_mapping_node.h"
#include "height_mapping/utils/config.h"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace height_mapping_ros {

HeightMappingNode::HeightMappingNode() : nh_("~") {

  // Configs from height_mapping_node.yaml
  ros::NodeHandle cfg_node(nh_, "node");
  ros::NodeHandle cfg_frame_id(nh_, "frame_id");
  ros::NodeHandle cfg_mapper(nh_, "height_mapper");

  // ROS node
  HeightMappingNode::loadConfig(cfg_node);
  initializePubSubs();
  initializeTimers();
  initializeServices();

  // TF frame IDs
  frame_ids::loadFromConfig(cfg_frame_id);

  // Height Mapper
  mapper_ = std::make_unique<height_mapping::HeightMapper>(
      height_mapper::loadConfig(cfg_mapper));

  std::cout << "\033[1;33m[height_mapping_ros::HeightMappingNode]: "
               "Height mapping node initialized. Waiting for scan inputs...\033[0m\n";
}

void HeightMappingNode::loadConfig(const ros::NodeHandle &nh) {

  cfg.lidarcloud_topic = nh.param<std::string>("lidar_topic", "/velodyne_points");
  cfg.rgbdcloud_topic = nh.param<std::string>("rgbd_topic", "/camera/pointcloud/points");
  cfg.pose_update_rate = nh.param<double>("pose_update_rate", 15.0);
  cfg.map_publish_rate = nh.param<double>("map_publish_rate", 10.0);
  cfg.remove_backward_points = nh.param<bool>("remove_backward_points", false);
  cfg.debug_mode = nh.param<bool>("debug_mode", false);
}

void HeightMappingNode::initializePubSubs() {

  sub_lidarscan_ =
      nh_.subscribe(cfg.lidarcloud_topic, 1, &HeightMappingNode::lidarScanCallback, this);
  sub_rgbdscan_ =
      nh_.subscribe(cfg.rgbdcloud_topic, 1, &HeightMappingNode::rgbdScanCallback, this);
  pub_lidarscan_rasterized_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/height_mapping/local/scan_rasterized_lidar",
      1);
  pub_rgbdscan_rasterized_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/height_mapping/local/scan_rasterized_rgbd",
      1);
  pub_heightmap_ = nh_.advertise<grid_map_msgs::GridMap>("/height_mapping/local/map", 1);

  if (cfg.debug_mode) {
    pub_debug_lidar_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/height_mapping/local/debug_lidar", 1);
    pub_debug_rgbd_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/height_mapping/local/debug_rgbd", 1);
  }
}

void HeightMappingNode::initializeTimers() {

  auto map_pose_update_duration = ros::Duration(1.0 / cfg.pose_update_rate);
  auto map_publish_duration = ros::Duration(1.0 / cfg.map_publish_rate);

  pose_update_timer_ = nh_.createTimer(map_pose_update_duration,
                                       &HeightMappingNode::updateMapOrigin,
                                       this,
                                       false,
                                       false);
  map_publish_timer_ = nh_.createTimer(map_publish_duration,
                                       &HeightMappingNode::publishHeightMap,
                                       this,
                                       false,
                                       false);
}

void HeightMappingNode::initializeServices() {

  // srv_save_map_ = nh_.advertiseService("/height_mapping/global/save_map",
  //  &GlobalMappingNode::saveMapCallback,
  //  this);
  // srv_clear_map_ = nh_.advertiseService("/height_mapping/global/clear_map",
  // &GlobalMappingNode::clearMapCallback,
  // this);
}

void HeightMappingNode::lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  if (!lidarscan_received_) {
    frame_ids::sensor::LIDAR = msg->header.frame_id;
    lidarscan_received_ = true;
    pose_update_timer_.start();
    map_publish_timer_.start();
    std::cout
        << "\033[1;32m[height_mapping_ros::HeightMappingNode]: Pointcloud Received! "
        << "Use LiDAR scans for height mapping... \033[0m\n";
  }

  // 1. Get transform matrix using tf tree
  geometry_msgs::TransformStamped lidar2base, base2map;
  if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, frame_ids::sensor::LIDAR, lidar2base) ||
      !tf_.lookupTransform(frame_ids::MAP, frame_ids::ROBOT_BASE, base2map))
    return;

  // 2. Convert ROS msg to PCL data
  auto scan_raw = boost::make_shared<pcl::PointCloud<Laser>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  // 3. Preprocess scan data: ready for terrain mapping
  auto scan_preprocessed = processLidarScan(scan_raw, lidar2base, base2map);
  if (!scan_preprocessed)
    return;

  // 4. Height mapping
  auto scan_rasterized = mapper_->heightMapping(scan_preprocessed);

  // 5. Publish pointcloud used for mapping
  publishRasterizedLidarScan(scan_rasterized);

  // 6. Raycasting correction: remove dynamic objects
  auto sensor2map = tf_.multiplyTransforms(lidar2base, base2map);
  Eigen::Vector3f sensorOrigin3D(sensor2map.transform.translation.x,
                                 sensor2map.transform.translation.y,
                                 sensor2map.transform.translation.z);
  mapper_->raycasting(sensorOrigin3D, scan_preprocessed);

  // Debug: publish pointcloud that you want to see
  if (cfg.debug_mode) {
    sensor_msgs::PointCloud2 msg_debug;
    pcl::toROSMsg(*scan_preprocessed, msg_debug);
    pub_debug_lidar_.publish(msg_debug);
  }
}

void HeightMappingNode::rgbdScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  // First time receiving RGB-D cloud -> start pose update timer
  if (!rgbdscan_received_) {
    rgbdscan_received_ = true;
    pose_update_timer_.start();
    map_publish_timer_.start();
    std::cout
        << "\033[1;33m[height_mapping_ros::HeightMappingNode]: Colored cloud Received! "
        << "Use RGB-D sensors for height mapping... \033[0m\n";
  }

  // Get Transform matrix
  frame_ids::sensor::RGBD = msg->header.frame_id;
  geometry_msgs::TransformStamped camera2base, base2map;
  if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, frame_ids::sensor::RGBD, camera2base) ||
      !tf_.lookupTransform(frame_ids::MAP, frame_ids::ROBOT_BASE, base2map))
    return;

  // Prepare pointcloud
  auto scan_raw = boost::make_shared<pcl::PointCloud<Color>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  // Preprocess pointcloud
  auto scan_processed = processRGBDScan(scan_raw, camera2base, base2map);
  if (!scan_processed)
    return;

  // Mapping
  auto cloud_mapped = mapper_->heightMapping<Color>(scan_processed);

  // Publish pointcloud used for mapping
  publishRasterizedRGBDScan(cloud_mapped);

  // Debug: publish pointcloud that you want to see
  if (cfg.debug_mode) {
    sensor_msgs::PointCloud2 debugMsg;
    pcl::toROSMsg(*scan_processed, debugMsg);
    pub_debug_rgbd_.publish(debugMsg);
  }
}

pcl::PointCloud<Laser>::Ptr
HeightMappingNode::processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                                    const geometry_msgs::TransformStamped &lidar2base,
                                    const geometry_msgs::TransformStamped &base2map) {
  // 1. Filter local pointcloud
  auto cloud_base = PointCloudOps::applyTransform<Laser>(cloud, lidar2base);
  auto cloud_processed = boost::make_shared<pcl::PointCloud<Laser>>();
  mapper_->fastHeightFilter(cloud_base, cloud_processed);
  auto range = mapper_->getHeightMap().getLength() / 2.0; // mapping range
  cloud_processed =
      PointCloudOps::passThrough<Laser>(cloud_processed, "x", -range.x(), range.x());
  cloud_processed =
      PointCloudOps::passThrough<Laser>(cloud_processed, "y", -range.y(), range.y());

  // (Optional) Remove remoter points
  if (cfg.remove_backward_points)
    cloud_processed = PointCloudOps::filterAngle2D<Laser>(cloud_processed, -135.0, 135.0);

  // 2. Transform pointcloud to map frame
  cloud_processed = PointCloudOps::applyTransform<Laser>(cloud_processed, base2map);

  if (cloud_processed->empty())
    return nullptr;
  return cloud_processed;
}

pcl::PointCloud<Color>::Ptr
HeightMappingNode::processRGBDScan(const pcl::PointCloud<Color>::Ptr &cloud,
                                   const geometry_msgs::TransformStamped &camera2base,
                                   const geometry_msgs::TransformStamped &base2map) {

  auto cloud_base = PointCloudOps::applyTransform<Color>(cloud, camera2base);
  auto cloud_processed = boost::make_shared<pcl::PointCloud<Color>>();
  mapper_->fastHeightFilter<Color>(cloud_base, cloud_processed);
  auto range = mapper_->getHeightMap().getLength() / 2.0; // mapping range
  cloud_processed =
      PointCloudOps::passThrough<Color>(cloud_processed, "x", -range.x(), range.x());
  cloud_processed =
      PointCloudOps::passThrough<Color>(cloud_processed, "y", -range.y(), range.y());

  // (Optional) Remove remoter points
  if (cfg.remove_backward_points)
    cloud_processed = PointCloudOps::filterAngle2D<Color>(cloud_processed, -135.0, 135.0);

  cloud_processed = PointCloudOps::applyTransform<Color>(cloud_processed, base2map);

  if (cloud_processed->empty())
    return nullptr;
  return cloud_processed;
}

void HeightMappingNode::updateMapOrigin(const ros::TimerEvent &event) {

  geometry_msgs::TransformStamped base2map;
  if (!tf_.lookupTransform(frame_ids::MAP, frame_ids::ROBOT_BASE, base2map))
    return;

  // Update map origin
  auto robot_position = grid_map::Position(base2map.transform.translation.x,
                                           base2map.transform.translation.y);
  mapper_->moveMapOrigin(robot_position);
}

void HeightMappingNode::publishHeightMap(const ros::TimerEvent &event) {

  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(mapper_->getHeightMap(), msg);
  pub_heightmap_.publish(msg);
}

void HeightMappingNode::publishRasterizedLidarScan(
    const pcl::PointCloud<Laser>::Ptr &scan) {

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*scan, msg);
  pub_lidarscan_rasterized_.publish(msg);
}

void HeightMappingNode::publishRasterizedRGBDScan(
    const pcl::PointCloud<Color>::Ptr &scan) {
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*scan, msg);
  pub_rgbdscan_rasterized_.publish(msg);
}

} // namespace height_mapping_ros

int main(int argc, char **argv) {

  ros::init(argc, argv, "height_mapping_node"); // launch file overrides the name
  height_mapping_ros::HeightMappingNode node;
  ros::spin();

  return 0;
}