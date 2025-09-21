/*
 * global_mapping.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/stages/global_mapping.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/stage_registry.h"

#include <iostream>

namespace height_mapping::core {

GlobalMappingStage::GlobalMappingStage()
    : Stage("GlobalMapping", "Global Map Aggregation") {
}

void GlobalMappingStage::configure(const std::map<std::string, std::string>& params) {
  auto it = params.find("global_map_size_x");
  if (it != params.end()) {
    global_map_size_x_ = std::stof(it->second);
  }

  it = params.find("global_map_size_y");
  if (it != params.end()) {
    global_map_size_y_ = std::stof(it->second);
  }

  it = params.find("global_map_resolution");
  if (it != params.end()) {
    global_map_resolution_ = std::stof(it->second);
    if (global_map_resolution_ <= 0) {
      std::cerr << "[GlobalMapping] Invalid resolution: " << global_map_resolution_
                << ", using default 0.1" << std::endl;
      global_map_resolution_ = 0.1f;
    }
  }

  it = params.find("merge_method");
  if (it != params.end()) {
    merge_method_ = it->second;
    if (merge_method_ != "averaging" && merge_method_ != "max_height" &&
        merge_method_ != "latest") {
      std::cerr << "[GlobalMapping] Unknown merge method: " << merge_method_
                << ", using 'averaging'" << std::endl;
      merge_method_ = "averaging";
    }
  }

  it = params.find("memory_limit_mb");
  if (it != params.end()) {
    memory_limit_mb_ = std::stoull(it->second);
  }

  it = params.find("enable_compression");
  if (it != params.end()) {
    enable_compression_ = (it->second == "true" || it->second == "1");
  }
}

void GlobalMappingStage::processImpl(pipeline::Context& ctx) {
  auto& mapping_ctx = static_cast<MappingContext&>(ctx);

  auto& local_map = mapping_ctx.map();
  if (local_map.empty()) {
    std::cerr << "[GlobalMapping] No local map in context" << std::endl;
    return;
  }

  // Check memory usage before proceeding
  if (!checkMemoryUsage()) {
    std::cerr << "[GlobalMapping] Memory limit exceeded, skipping update" << std::endl;
    return;
  }

  // Merge local map into global map
  mergeLocalToGlobal(ctx);

  // Update statistics
  total_local_maps_integrated_++;
  global_map_coverage_percent_ = calculateCoverage();

  // Store statistics in context

  // Calculate global map dimensions
  int global_cells_x = static_cast<int>(global_map_size_x_ / global_map_resolution_);
  int global_cells_y = static_cast<int>(global_map_size_y_ / global_map_resolution_);
}

void GlobalMappingStage::mergeLocalToGlobal(pipeline::Context& ctx) {
  // This is a simplified placeholder for the actual merge logic
  // In reality, this would:
  // 1. Get the local map position in global coordinates
  // 2. Find overlapping cells between local and global maps
  // 3. Apply the merge method (averaging, max, latest)
  // 4. Update global map cells

  // Simulate some cells being updated
  size_t cells_updated = 1000;  // Placeholder
  global_map_cells_updated_ += cells_updated;

  // Store merge results
}

float GlobalMappingStage::calculateCoverage() const {
  // Calculate what percentage of the global map has been covered
  // This is a simplified calculation
  int total_cells = static_cast<int>((global_map_size_x_ / global_map_resolution_) *
                                     (global_map_size_y_ / global_map_resolution_));

  if (total_cells > 0) {
    float coverage = static_cast<float>(global_map_cells_updated_) / total_cells * 100.0f;
    return std::min(coverage, 100.0f);
  }

  return 0.0f;
}

bool GlobalMappingStage::checkMemoryUsage() const {
  // Check if memory usage is within limits
  // This is a simplified check
  size_t estimated_memory_mb = (global_map_cells_updated_ * sizeof(float) * 4) / (1024 * 1024);

  if (enable_compression_) {
    // Assume compression reduces size by ~50%
    estimated_memory_mb /= 2;
  }

  return estimated_memory_mb < memory_limit_mb_;
}

// Register this stage with the factory
REGISTER_STAGE(GlobalMappingStage)

} // namespace height_mapping::core