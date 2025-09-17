/*
 * sensor_processor_node.cpp
 *
 *  Created on: Nov 25, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/sensor_processor_node.h"
#include "height_mapping/utils/config.h"

#include <pcl_conversions/pcl_conversions.h>

namespace height_mapping_ros {

SensorProcessorNode::SensorProcessorNode() : nh_("~") {

  // Configs from sensor_processor_node.yaml
  ros::NodeHandle cfg_node(nh_, "node");
  ros::NodeHandle cfg_frame_id(nh_, "frame_id");

  // ROS node
  SensorProcessorNode::loadConfig(cfg_node);
  initializePubSubs();

  // TF frame IDs
  frame_ids::loadFromConfig(cfg_frame_id);

  std::cout << "\033[1;32m[height_mapping_ros::SensorProcessorNode]: "
               "Sensor processor node initialized. Waiting for "
            << cfg.inputcloud_topics.size() << " rgb clouds... \033[0m\n";
}

void SensorProcessorNode::loadConfig(const ros::NodeHandle &nh) {

  cfg.inputcloud_topics = nh.param<std::vector<std::string>>(
      "input_cloud_topics",
      {"/sensor1/points", "/sensor2/points", "/sensor3/points"});
  cfg.outputcloud_topic =
      nh.param<std::string>("outputcloud_topic", "/sensor_processor/points");
  cfg.cloud_publish_rate = nh.param<double>("cloud_publish_rate", 10.0);      // [Hz]
  cfg.downsample_resolution = nh.param<double>("downsample_resolution", 0.1); // [m/grid]
  cfg.min_range_threshold = nh.param<double>("min_range_threshold", 0.3);     // [m]
  cfg.max_range_threshold = nh.param<double>("max_range_threshold", 5.0);     // [m]
}

void SensorProcessorNode::initializePubSubs() {

  // Subscriber with synchronizer
  auto queue_size = 10;
  for (const auto &topic : cfg.inputcloud_topics) {
    sub_clouds_.push_back(std::make_shared<CloudSubscriber>(nh_, topic, queue_size));
  }
  if (sub_clouds_.size() == 2) {
    sync2_.reset(
        new Synchronizer2(SyncPolicy2(queue_size), *sub_clouds_[0], *sub_clouds_[1]));
    sync2_->registerCallback(
        boost::bind(&SensorProcessorNode::syncCallback2, this, _1, _2));
  } else if (sub_clouds_.size() == 3) {
    sync3_.reset(new Synchronizer3(SyncPolicy3(queue_size),
                                   *sub_clouds_[0],
                                   *sub_clouds_[1],
                                   *sub_clouds_[2]));
    sync3_->registerCallback(
        boost::bind(&SensorProcessorNode::syncCallback3, this, _1, _2, _3));
  }

  // Publisher
  pub_cloud_processed_ =
      nh_.advertise<sensor_msgs::PointCloud2>(cfg.outputcloud_topic, 1);
}

void SensorProcessorNode::syncCallback2(const sensor_msgs::PointCloud2ConstPtr &msg1,
                                        const sensor_msgs::PointCloud2ConstPtr &msg2) {

  // Convert to PCL (be careful with const_cast)
  pcl::moveFromROSMsg(*const_cast<sensor_msgs::PointCloud2 *>(msg1.get()), *cloud1_);
  pcl::moveFromROSMsg(*const_cast<sensor_msgs::PointCloud2 *>(msg2.get()), *cloud2_);

  // Downsampling
  downsampled1_ =
      PointCloudOps::downsampleVoxel<Color>(cloud1_, cfg.downsample_resolution);
  downsampled2_ =
      PointCloudOps::downsampleVoxel<Color>(cloud2_, cfg.downsample_resolution);

  // Free memory
  cloud1_->clear();
  cloud2_->clear();

  // Transform each cloud to baselink frame
  transformToBaselink(downsampled1_, transformed1_, msg1->header.frame_id);
  transformToBaselink(downsampled2_, transformed2_, msg2->header.frame_id);

  // Free memory
  downsampled1_->clear();
  downsampled2_->clear();

  // Filter each cloud
  filtered1_ = PointCloudOps::filterRange2D<Color>(transformed1_,
                                                   cfg.min_range_threshold,
                                                   cfg.max_range_threshold);
  filtered2_ = PointCloudOps::filterRange2D<Color>(transformed2_,
                                                   cfg.min_range_threshold,
                                                   cfg.max_range_threshold);

  // Free memory
  transformed1_->clear();
  transformed2_->clear();

  // Merge transformed clouds
  *filtered1_ += *filtered2_;

  // Convert back to ROS message
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*filtered1_, msg);
  msg.header.frame_id = frame_ids::ROBOT_BASE;
  msg.header.stamp = msg1->header.stamp; // Use timestamp from first message
  pub_cloud_processed_.publish(msg);

  // Free memory
  filtered1_->clear();
  filtered2_->clear();
}

void SensorProcessorNode::syncCallback3(const sensor_msgs::PointCloud2ConstPtr &msg1,
                                        const sensor_msgs::PointCloud2ConstPtr &msg2,
                                        const sensor_msgs::PointCloud2ConstPtr &msg3) {

  // Convert to PCL (be careful with const_cast)
  pcl::moveFromROSMsg(*const_cast<sensor_msgs::PointCloud2 *>(msg1.get()), *cloud1_);
  pcl::moveFromROSMsg(*const_cast<sensor_msgs::PointCloud2 *>(msg2.get()), *cloud2_);
  pcl::moveFromROSMsg(*const_cast<sensor_msgs::PointCloud2 *>(msg3.get()), *cloud3_);

  // Downsampling
  downsampled1_ =
      PointCloudOps::downsampleVoxel<Color>(cloud1_, cfg.downsample_resolution);
  downsampled2_ =
      PointCloudOps::downsampleVoxel<Color>(cloud2_, cfg.downsample_resolution);
  downsampled3_ =
      PointCloudOps::downsampleVoxel<Color>(cloud3_, cfg.downsample_resolution);

  // Free memory
  cloud1_->clear();
  cloud2_->clear();
  cloud3_->clear();

  // Transform each cloud to baselink frame
  transformToBaselink(downsampled1_, transformed1_, msg1->header.frame_id);
  transformToBaselink(downsampled2_, transformed2_, msg2->header.frame_id);
  transformToBaselink(downsampled3_, transformed3_, msg3->header.frame_id);

  // Free memory
  downsampled1_->clear();
  downsampled2_->clear();
  downsampled3_->clear();

  // Filter each cloud
  filtered1_ = PointCloudOps::filterRange2D<Color>(transformed1_,
                                                   cfg.min_range_threshold,
                                                   cfg.max_range_threshold);
  filtered2_ = PointCloudOps::filterRange2D<Color>(transformed2_,
                                                   cfg.min_range_threshold,
                                                   cfg.max_range_threshold);
  filtered3_ = PointCloudOps::filterRange2D<Color>(transformed3_,
                                                   cfg.min_range_threshold,
                                                   cfg.max_range_threshold);

  // Free memory
  transformed1_->clear();
  transformed2_->clear();
  transformed3_->clear();

  // Merge transformed clouds
  *filtered1_ += *filtered2_;
  *filtered1_ += *filtered3_;

  // Convert back to ROS message
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(*filtered1_, msg);
  msg.header.frame_id = frame_ids::ROBOT_BASE;
  msg.header.stamp = msg1->header.stamp;
  pub_cloud_processed_.publish(msg);

  // Free memory
  filtered1_->clear();
  filtered2_->clear();
  filtered3_->clear();
}

void SensorProcessorNode::transformToBaselink(
    const pcl::PointCloud<Color>::Ptr &cloud,
    pcl::PointCloud<Color>::Ptr &cloud_transformed,
    const std::string &sensor_frame) {

  // Get transform matrix
  geometry_msgs::TransformStamped transform;
  if (!tf_.lookupTransform(frame_ids::ROBOT_BASE, sensor_frame, transform))
    return;

  // Transform point cloud
  cloud_transformed = PointCloudOps::applyTransform<Color>(cloud, transform);
}

} // namespace height_mapping_ros

int main(int argc, char **argv) {

  ros::init(argc, argv, "sensor_processor_node");
  height_mapping_ros::SensorProcessorNode node;
  ros::spin();

  return 0;
}