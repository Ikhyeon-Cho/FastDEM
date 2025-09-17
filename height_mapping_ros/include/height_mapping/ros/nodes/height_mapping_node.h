/*
 * height_mapping_node.h
 *
 *  Created on: Nov 24, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "height_mapping/ros/ros_common.h"
#include "height_mapping_core/HeightMapper.h"
#include "height_mapping_core/GlobalMapper.h"

namespace height_mapping_ros {

class HeightMappingNode {
public:
  struct Config {
    std::string lidarcloud_topic;
    std::string rgbdcloud_topic;
    double pose_update_rate;
    double map_publish_rate;
    bool remove_backward_points;
    bool debug_mode;
  } cfg;

  HeightMappingNode();
  ~HeightMappingNode() = default;
  void loadConfig(const ros::NodeHandle &nh);

private:
  // init functions
  void initializeTimers();
  void initializePubSubs();
  void initializeServices();

  // callback functions
  void lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg);
  void rgbdScanCallback(const sensor_msgs::PointCloud2Ptr &msg);

  // Core logics
  pcl::PointCloud<Laser>::Ptr
  processLidarScan(const pcl::PointCloud<Laser>::Ptr &cloud,
                   const geometry_msgs::TransformStamped &lidar2base,
                   const geometry_msgs::TransformStamped &base2map);
  pcl::PointCloud<Color>::Ptr
  processRGBDScan(const pcl::PointCloud<Color>::Ptr &cloud,
                  const geometry_msgs::TransformStamped &camera2base,
                  const geometry_msgs::TransformStamped &base2map);
  void publishRasterizedLidarScan(const pcl::PointCloud<Laser>::Ptr &scan);
  void publishRasterizedRGBDScan(const pcl::PointCloud<Color>::Ptr &scan);
  void updateMapOrigin(const ros::TimerEvent &event);
  void publishHeightMap(const ros::TimerEvent &event);

  ros::NodeHandle nh_;

  // Subscribers & Publishers
  ros::Subscriber sub_lidarscan_;
  ros::Subscriber sub_rgbdscan_;
  ros::Publisher pub_lidarscan_rasterized_;
  ros::Publisher pub_rgbdscan_rasterized_;
  ros::Publisher pub_heightmap_;
  ros::Publisher pub_debug_lidar_;
  ros::Publisher pub_debug_rgbd_;

  // Timers
  ros::Timer pose_update_timer_;
  ros::Timer map_publish_timer_;

  // Core objects
  std::unique_ptr<height_mapping::HeightMapper> mapper_;
  TransformOps tf_;

  // State variables
  bool lidarscan_received_{false};
  bool rgbdscan_received_{false};
};
} // namespace height_mapping_ros