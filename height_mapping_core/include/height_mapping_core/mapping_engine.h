#pragma once

#include "height_mapping_core/data/height_map.h"
#include "height_mapping_core/data/point_cloud.h"
#include "height_mapping_core/interfaces/transform_provider.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/pipeline.h"
#include <atomic>
#include <memory>
#include <mutex>
#include <shared_mutex>

namespace height_mapping::core {

// Import types from height_map namespace
using PointCloudXYZ = height_map::PointCloudXYZ;

// Simple configuration structures
struct EngineConfig {
  // Map configuration (creation and management)
  struct Map {
    double width = 10.0;     // meters
    double height = 10.0;    // meters
    double resolution = 0.1; // meters per cell
    std::string frame_id = "map";
  } map;

  // Engine-specific behavior
  struct Engine {
    bool thread_safe = true;
    bool auto_reset_on_error = false;
    size_t max_queue_size = 10;
    bool enable_statistics = false;
  } engine;

  // Pipeline configuration file path
  std::string pipeline_config_file;
};

class MappingEngine {
public:
  // Clean interface: configuration + optional transform provider
  MappingEngine(
      const EngineConfig &config,
      std::shared_ptr<ITransformProvider> transform_provider = nullptr);

  // Factory method: Engine owns config loading responsibility
  static std::unique_ptr<MappingEngine> createFromFile(
      const std::string &config_file,
      std::shared_ptr<ITransformProvider> transform_provider = nullptr);

  ~MappingEngine() = default;

  // Main processing interface
  void integrateCloud(const PointCloudXYZ &cloud); // Copy version
  void integrateCloud(PointCloudXYZ &&cloud); // Move version (for performance)

  // Data access interfaces
  std::shared_ptr<const height_map::HeightMap> getHeightMap() const;
  std::shared_ptr<const PointCloudXYZ> getProcessedCloud() const;

  // Control interfaces
  void reset();
  bool isInitialized() const { return initialized_; }

  // Statistics (if enabled)
  struct Statistics {
    size_t clouds_processed = 0;
    double avg_processing_time_ms = 0.0;
    double variance_processing_time_ms = 0.0; // Added for std deviation
    size_t pipeline_errors = 0;
  };
  Statistics getStatistics() const;

private:
  // Common implementation for both copy and move versions
  void integrateCloudImpl(MappingContext &ctx);

  void setupPipeline();
  void initializeMap();
  void updateStatistics(double duration_ms);

  EngineConfig config_;
  std::shared_ptr<ITransformProvider> transform_provider_;

  std::shared_ptr<height_map::HeightMap> map_;
  std::unique_ptr<pipeline::Pipeline> mapping_pipeline_;

  mutable std::shared_mutex
      map_mutex_; // Changed to shared_mutex for read/write locks
  mutable std::shared_mutex cloud_mutex_; // Changed to shared_mutex
  std::shared_ptr<PointCloudXYZ> last_processed_cloud_;

  // Statistics with Welford's algorithm for numerical stability
  struct WelfordStats {
    std::atomic<size_t> count{0};
    std::atomic<double> mean{0.0};
    std::atomic<double> M2{0.0}; // Sum of squares of differences from mean
    std::atomic<size_t> errors{0};
  } welford_stats_;

  bool initialized_ = false;
};

} // namespace height_mapping::core