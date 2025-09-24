/*
 * voxel_filter.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/voxel_filter.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registry.h"

#include <cmath>
#include <limits>
#include <logger/logger.h>
#include <map>
#include <random>

namespace height_mapping::core::stages {

using PointCloud = geometry::PointCloud;
using Point3D = geometry::Point3D;

VoxelFilter::VoxelFilter() : Stage("VoxelFilter") {}

void VoxelFilter::configure(const std::map<std::string, std::string> &params) {
  // Configure voxel size
  auto it = params.find("voxel_size");
  if (it != params.end()) {
    float size = std::stof(it->second);
    if (size > 0.0f && size <= 10.0f) {
      voxel_size_ = size;
    } else {
      LOG_ERROR("VoxelFilter", "Invalid voxel size: ", size);
    }
  }

  // Configure reduction method
  it = params.find("reduction_method");
  if (it != params.end()) {
    const std::string &method = it->second;
    if (method == "centroid") {
      reduction_method_ = ReductionMethod::CENTROID;
    } else if (method == "random") {
      reduction_method_ = ReductionMethod::RANDOM;
    } else if (method == "first") {
      reduction_method_ = ReductionMethod::FIRST;
    } else if (method == "closest") {
      reduction_method_ = ReductionMethod::CLOSEST;
    } else if (method == "center" || method == "voxel_center") {
      reduction_method_ = ReductionMethod::VOXEL_CENTER;
    } else {
      LOG_ERROR("VoxelFilter", "Unknown reduction method: ", method);
    }
  }
}

void VoxelFilter::processImpl(pipeline::Context &ctx) {
  auto &mapping_ctx = static_cast<MappingContext &>(ctx);
  auto &cloud = mapping_ctx.cloud();
  if (cloud.empty()) {
    return;
  }

  // Store sensor data availability
  const bool has_intensity = cloud.hasIntensity();
  const bool has_color = cloud.hasColor();

  // Group points by voxel with indices for sensor data tracking
  struct VoxelData {
    std::vector<Point3D> points;
    std::vector<size_t> indices; // Original indices for sensor data
  };
  std::map<VoxelKey, VoxelData> voxel_map;

  for (size_t i = 0; i < cloud.size(); ++i) {
    auto point = cloud[i]; // PointView access

    // Skip invalid points
    if (!point.isFinite()) {
      continue;
    }

    // Compute voxel index
    VoxelKey key = pointToVoxel(point.x(), point.y(), point.z());
    voxel_map[key].points.push_back(point.xyz());
    voxel_map[key].indices.push_back(i);
  }

  // Create filtered cloud with same sensor data configuration
  PointCloud filtered_cloud;
  filtered_cloud.reserve(voxel_map.size());
  filtered_cloud.setFrameId(cloud.frameId());
  filtered_cloud.setTimestamp(cloud.timestamp());

  // Pre-allocate sensor data arrays if needed
  if (has_intensity) {
    std::vector<float> intensity_data;
    intensity_data.reserve(voxel_map.size());
    filtered_cloud.attachIntensity(std::move(intensity_data));
  }
  if (has_color) {
    std::vector<std::array<uint8_t, 3>> color_data;
    color_data.reserve(voxel_map.size());
    filtered_cloud.attachColor(std::move(color_data));
  }

  for (const auto &[voxel_key, voxel_data] : voxel_map) {
    Point3D representative;
    size_t selected_index; // Index for sensor data

    // Apply reduction method and select sensor data source
    switch (reduction_method_) {
    case ReductionMethod::CENTROID:
      representative = reduceCentroid(voxel_data.points);
      // Use first point's sensor data
      selected_index = voxel_data.indices[0];
      break;

    case ReductionMethod::RANDOM: {
      size_t random_idx = std::rand() % voxel_data.points.size();
      representative = voxel_data.points[random_idx];
      selected_index = voxel_data.indices[random_idx];
    } break;

    case ReductionMethod::FIRST:
      representative = reduceFirst(voxel_data.points);
      selected_index = voxel_data.indices[0];
      break;

    case ReductionMethod::CLOSEST:
      representative = reduceClosest(voxel_data.points, voxel_key);
      // Find which point was selected as closest
      selected_index = voxel_data.indices[0]; // Default to first
      for (size_t i = 0; i < voxel_data.points.size(); ++i) {
        if ((voxel_data.points[i] - representative).squaredNorm() < 1e-6) {
          selected_index = voxel_data.indices[i];
          break;
        }
      }
      break;

    case ReductionMethod::VOXEL_CENTER:
      representative = reduceVoxelCenter(voxel_key);
      // Use first point's sensor data for synthetic center
      selected_index = voxel_data.indices[0];
      break;
    }

    // Add representative point with its sensor data
    auto selected_point = cloud[selected_index];
    filtered_cloud.addPoint(representative, selected_point.intensity(),
                            selected_point.color());
  }

  // Replace cloud with filtered version
  cloud = std::move(filtered_cloud);
}

// Helper implementations
VoxelFilter::VoxelKey VoxelFilter::pointToVoxel(float x, float y,
                                                float z) const {
  return VoxelKey{static_cast<int>(std::floor(x / voxel_size_)),
                  static_cast<int>(std::floor(y / voxel_size_)),
                  static_cast<int>(std::floor(z / voxel_size_))};
}

Point3D VoxelFilter::reduceCentroid(const std::vector<Point3D> &points) const {
  Point3D centroid{0, 0, 0};
  for (const auto &p : points) {
    centroid.x += p.x;
    centroid.y += p.y;
    centroid.z += p.z;
  }
  float n = static_cast<float>(points.size());
  centroid.x /= n;
  centroid.y /= n;
  centroid.z /= n;
  return centroid;
}

Point3D VoxelFilter::reduceRandom(const std::vector<Point3D> &points) const {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, points.size() - 1);
  return points[dis(gen)];
}

Point3D VoxelFilter::reduceFirst(const std::vector<Point3D> &points) const {
  return points.front();
}

Point3D VoxelFilter::reduceClosest(const std::vector<Point3D> &points,
                                   const VoxelKey &voxel) const {
  // Calculate voxel center
  float cx = (voxel.x + 0.5f) * voxel_size_;
  float cy = (voxel.y + 0.5f) * voxel_size_;
  float cz = (voxel.z + 0.5f) * voxel_size_;

  // Find point closest to center
  float min_dist_sq = std::numeric_limits<float>::max();
  Point3D closest = points.front();

  for (const auto &p : points) {
    float dx = p.x - cx;
    float dy = p.y - cy;
    float dz = p.z - cz;
    float dist_sq = dx * dx + dy * dy + dz * dz;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest = p;
    }
  }
  return closest;
}

Point3D VoxelFilter::reduceVoxelCenter(const VoxelKey &voxel) const {
  return Point3D{(voxel.x + 0.5f) * voxel_size_, (voxel.y + 0.5f) * voxel_size_,
                 (voxel.z + 0.5f) * voxel_size_};
}

// Register this stage with the factory
REGISTER_STAGE(VoxelFilter)

} // namespace height_mapping::core::stages