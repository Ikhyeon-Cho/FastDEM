/*
 * snapshot.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_IO_SNAPSHOT_H
#define HEIGHT_MAPPING_IO_SNAPSHOT_H

#include <sys/stat.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <nanopcl/io.hpp>
#include <sstream>

#include "height_mapping/core.h"
#include "height_mapping/io.h"

namespace height_mapping {
namespace io {

namespace detail {

inline bool createDirectory(const std::string& path) {
  return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

inline std::string generateTimestampDir(
    const std::string& base_dir, const std::string& prefix = "snapshot") {
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto tm = *std::localtime(&time_t);

  std::ostringstream oss;
  oss << base_dir << "/" << prefix << "_"
      << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

}  // namespace detail

/**
 * @brief Save complete mapping state to a directory
 *
 * Creates a snapshot directory containing:
 * - cloud.pcd: Point cloud with intensity (if available)
 * - heightmap.hmap: Binary height map with all layers
 * - robot_pose.tum: Robot pose in TUM format
 *
 * Directory structure:
 *   snapshot_YYYYMMDD_HHMMSS/
 *   ├── cloud.pcd
 *   ├── heightmap.hmap
 *   └── robot_pose.tum
 *
 * @param directory Base directory (snapshot subfolder will be created)
 * @param cloud Point cloud to save
 * @param map Height map to save
 * @param robot_pose Robot pose (Eigen::Isometry)
 * @param timestamp_sec Timestamp for TUM format (0 = use current time)
 * @return Path to created snapshot directory, or empty string on failure
 */
template <typename Scalar = double>
inline std::string saveSnapshot(
    const std::string& directory, const npcl::PointCloud& cloud,
    const HeightMap& map,
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& robot_pose,
    double timestamp_sec = 0.0) {
  // Generate timestamp directory
  std::string snapshot_dir = detail::generateTimestampDir(directory);

  // Create directories
  if (!detail::createDirectory(directory)) {
    std::cerr << "[snapshot] Error: Cannot create base directory " << directory
              << std::endl;
    return "";
  }
  if (!detail::createDirectory(snapshot_dir)) {
    std::cerr << "[snapshot] Error: Cannot create snapshot directory "
              << snapshot_dir << std::endl;
    return "";
  }

  // Use current time if not specified
  if (timestamp_sec <= 0.0) {
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    timestamp_sec = std::chrono::duration<double>(duration).count();
  }

  bool success = true;

  // Save point cloud
  std::string cloud_path = snapshot_dir + "/cloud.pcd";
  try {
    npcl::io::savePCD(cloud_path, cloud);
  } catch (const std::exception& e) {
    std::cerr << "[snapshot] Warning: Failed to save cloud.pcd: " << e.what()
              << std::endl;
    success = false;
  }

  // Save height map
  std::string map_path = snapshot_dir + "/heightmap.hmap";
  if (!io::saveHMap(map_path, map)) {
    std::cerr << "[snapshot] Warning: Failed to save heightmap.hmap"
              << std::endl;
    success = false;
  }

  // Save robot pose as TUM format
  std::string pose_path = snapshot_dir + "/robot_pose.tum";
  try {
    // Convert Eigen::Isometry to npcl::Transform
    npcl::Transform_<Scalar> tf("map", "base_link", robot_pose);
    tf.setTimestamp(npcl::time::fromSec(timestamp_sec));
    npcl::io::saveTransformTUM(pose_path, tf);
  } catch (const std::exception& e) {
    std::cerr << "[snapshot] Warning: Failed to save robot_pose.tum: "
              << e.what() << std::endl;
    success = false;
  }

  if (!success) {
    std::cerr << "[snapshot] Some files failed to save" << std::endl;
  }

  return snapshot_dir;
}

/**
 * @brief Save complete mapping state with visualization images
 *
 * Creates everything in saveSnapshot() plus a viz/ subfolder with PPM images.
 *
 * Directory structure:
 *   snapshot_YYYYMMDD_HHMMSS/
 *   ├── cloud.pcd
 *   ├── heightmap.hmap
 *   ├── robot_pose.tum
 *   └── viz/
 *       └── elevation.ppm
 *
 * @param directory Base directory
 * @param cloud Point cloud to save
 * @param map Height map to save
 * @param robot_pose Robot pose (Eigen::Isometry)
 * @param viz_config Configuration for visualization export
 * @param viz_layers Layers to export as images (empty = elevation)
 * @param timestamp_sec Timestamp for TUM format (0 = use current time)
 * @return Path to created snapshot directory, or empty string on failure
 */
template <typename Scalar = double>
inline std::string saveSnapshotWithViz(
    const std::string& directory, const npcl::PointCloud& cloud,
    const HeightMap& map,
    const Eigen::Transform<Scalar, 3, Eigen::Isometry>& robot_pose,
    const io::PpmExportConfig& viz_config = {},
    const std::vector<std::string>& viz_layers = {},
    double timestamp_sec = 0.0) {
  // First save the basic snapshot
  std::string snapshot_dir =
      saveSnapshot(directory, cloud, map, robot_pose, timestamp_sec);

  if (snapshot_dir.empty()) {
    return "";
  }

  // Create viz directory
  std::string viz_dir = snapshot_dir + "/viz";
  if (!detail::createDirectory(viz_dir)) {
    std::cerr << "[snapshot] Warning: Cannot create viz directory" << std::endl;
    return snapshot_dir;
  }

  // Determine which layers to visualize
  std::vector<std::string> layers = viz_layers;
  if (layers.empty()) {
    // Default: elevation only
    layers = {layer::elevation};
  }

  // Export each layer as PPM
  for (const auto& layer_name : layers) {
    if (!map.exists(layer_name)) {
      continue;
    }

    std::string image_path = viz_dir + "/" + layer_name + ".ppm";
    if (!io::savePpm(image_path, map, layer_name, viz_config)) {
      std::cerr << "[snapshot] Warning: Failed to export " << layer_name
                << ".ppm" << std::endl;
    }
  }

  return snapshot_dir;
}

/**
 * @brief Load snapshot from directory
 *
 * Loads all components of a snapshot:
 * - Point cloud from cloud.pcd
 * - Height map from heightmap.hmap
 * - Robot pose from robot_pose.tum
 *
 * @param snapshot_dir Path to snapshot directory
 * @param cloud [out] Loaded point cloud
 * @param map [out] Loaded height map
 * @param robot_pose [out] Loaded robot pose
 * @param timestamp_sec [out] Loaded timestamp
 * @return true if all components loaded successfully
 */
template <typename Scalar = double>
inline bool loadSnapshot(
    const std::string& snapshot_dir, npcl::PointCloud& cloud, HeightMap& map,
    Eigen::Transform<Scalar, 3, Eigen::Isometry>& robot_pose,
    double& timestamp_sec) {
  bool success = true;

  // Load point cloud
  std::string cloud_path = snapshot_dir + "/cloud.pcd";
  try {
    cloud = npcl::io::loadPCD(cloud_path);
  } catch (const std::exception& e) {
    std::cerr << "[snapshot] Warning: Failed to load cloud.pcd: " << e.what()
              << std::endl;
    success = false;
  }

  // Load height map
  std::string map_path = snapshot_dir + "/heightmap.hmap";
  if (!io::loadHMap(map_path, map)) {
    std::cerr << "[snapshot] Warning: Failed to load heightmap.hmap"
              << std::endl;
    success = false;
  }

  // Load robot pose
  std::string pose_path = snapshot_dir + "/robot_pose.tum";
  try {
    auto tf = npcl::io::loadTransformTUM<Scalar>(pose_path, "map", "base_link");
    robot_pose = tf.pose();
    timestamp_sec = npcl::time::toSec(tf.timestamp());
  } catch (const std::exception& e) {
    std::cerr << "[snapshot] Warning: Failed to load robot_pose.tum: "
              << e.what() << std::endl;
    success = false;
  }

  return success;
}

}  // namespace io
}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_IO_SNAPSHOT_H
