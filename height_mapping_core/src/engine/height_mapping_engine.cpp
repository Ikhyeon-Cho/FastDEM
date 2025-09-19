/*
 * height_mapping_engine.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/engine/height_mapping_engine.h"
#include "height_mapping_core/config/config_loader.h"
#include "height_mapping_core/pipeline/pipeline_builder.h"
#include <chrono>

namespace height_mapping::core {

MappingEngine::MappingEngine(
    const EngineConfig &config,
    std::shared_ptr<ITransformProvider> transform_provider)
    : config_(config), transform_provider_(transform_provider) {

  initializeMap();
  setupPipeline();
  initialized_ = true;
}

// Factory method: Engine owns config loading responsibility
std::unique_ptr<MappingEngine> MappingEngine::createFromFile(
    const std::string &config_file,
    std::shared_ptr<ITransformProvider> transform_provider) {

  // Engine loads its own configuration
  auto config = ConfigLoader::loadEngineConfig(config_file);

  // Create engine with loaded config
  return std::make_unique<MappingEngine>(config, transform_provider);
}

void MappingEngine::initializeMap() {
  // Create and initialize the height map
  height_map_ = std::make_shared<height_map::HeightMap>();
  height_map_->initialize(config_.map.width, config_.map.height,
                          config_.map.resolution);
  height_map_->setFrameId(config_.map.frame_id);
}

void MappingEngine::setupPipeline() {
  // Use PipelineBuilder to create and configure the pipeline
  mapping_pipeline_ = PipelineBuilder()
                          .fromConfig(config_.pipeline)
                          .withTransformProvider(transform_provider_)
                          .build();
}

void MappingEngine::integrateCloud(const PointCloudXYZ &cloud) {
  ProcessingContext ctx;
  ctx.cloud() = cloud;
  integrateCloudImpl(ctx);
}

void MappingEngine::integrateCloud(PointCloudXYZ &&cloud) {
  ProcessingContext ctx;
  ctx.cloud() = std::move(cloud);
  integrateCloudImpl(ctx);
}

void MappingEngine::integrateCloudImpl(ProcessingContext &ctx) {

  if (!initialized_) {
    throw std::runtime_error(
        "MappingEngine::integrateCloud() called before initialization. "
        "Ensure MappingEngine constructor completed successfully.");
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  // Process through pipeline
  try {
    {
      // Process through pipeline while holding the lock
      // This ensures thread-safe map modifications
      std::unique_lock<std::shared_mutex> lock(map_mutex_);
      ctx.setMap(height_map_);
      mapping_pipeline_->process(ctx);
    }

    // Store processed cloud (write lock needed)
    {
      std::unique_lock<std::shared_mutex> lock(cloud_mutex_);
      last_processed_cloud_ = ctx.getCloudPtr();
    }

    // Update statistics if enabled using Welford's algorithm
    if (config_.engine.enable_statistics) {
      auto end_time = std::chrono::high_resolution_clock::now();
      auto duration_ms = std::chrono::duration_cast<std::chrono::microseconds>(
                             end_time - start_time)
                             .count() /
                         1000.0;

      updateStatistics(duration_ms);
    }
  } catch (const std::exception &e) {
    if (config_.engine.enable_statistics) {
      welford_stats_.errors.fetch_add(1);
    }
    if (config_.engine.auto_reset_on_error) {
      reset();
    }
    throw std::runtime_error(
        "Pipeline processing failed in MappingEngine::integrateCloud(): " +
        std::string(e.what()) +
        " (Cloud size: " + std::to_string(ctx.cloud().points.size()) +
        " points, frame_id: " + ctx.cloud().frame_id + ")");
  }
}

std::shared_ptr<const height_map::HeightMap>
MappingEngine::getHeightMap() const {
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  return height_map_;
}

std::shared_ptr<const PointCloudXYZ> MappingEngine::getProcessedCloud() const {
  std::shared_lock<std::shared_mutex> lock(cloud_mutex_);
  return last_processed_cloud_;
}

void MappingEngine::reset() {
  // Use scoped_lock to avoid deadlock (locks both mutexes atomically)
  std::scoped_lock lock(map_mutex_, cloud_mutex_);

  // Reset the height map
  if (height_map_) {
    height_map_->clear();
  }

  // Clear processed cloud
  last_processed_cloud_.reset();

  // Reset statistics if enabled
  if (config_.engine.enable_statistics) {
    welford_stats_.count.store(0);
    welford_stats_.mean.store(0.0);
    welford_stats_.M2.store(0.0);
    welford_stats_.errors.store(0);
  }
}

MappingEngine::Statistics MappingEngine::getStatistics() const {
  Statistics stats;
  stats.clouds_processed = welford_stats_.count.load();
  stats.pipeline_errors = welford_stats_.errors.load();
  stats.avg_processing_time_ms = welford_stats_.mean.load();

  // Calculate variance/std deviation if we have enough samples
  size_t n = stats.clouds_processed;
  if (n > 1) {
    stats.variance_processing_time_ms = welford_stats_.M2.load() / (n - 1);
  }

  return stats;
}

void MappingEngine::updateStatistics(double duration_ms) {
  // Welford's online algorithm for numerical stability (lock-free)
  size_t n = welford_stats_.count.fetch_add(1) + 1;
  double delta = duration_ms - welford_stats_.mean.load();
  double new_mean = welford_stats_.mean.load() + delta / n;
  double delta2 = duration_ms - new_mean;

  welford_stats_.mean.store(new_mean);
  // For atomic<double>, use compare_exchange loop instead of fetch_add
  double current_M2 = welford_stats_.M2.load();
  while (!welford_stats_.M2.compare_exchange_weak(current_M2,
                                                  current_M2 + delta * delta2))
    ;
}

} // namespace height_mapping::core