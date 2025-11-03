#pragma once

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>

#include "flowpipe/pipeline.h"
#include "flowpipe/pipeline_profiler.h"
#include "height_mapping/core/data/height_map.h"
#include "height_mapping/core/data/point_cloud.h"
#include "height_mapping/pipeline/interfaces/config.h"
#include "height_mapping/pipeline/interfaces/transform_lookup.h"

namespace height_mapping {

class HeightMapper {
 public:
  using Ptr = std::unique_ptr<HeightMapper>;

  // Constructor with optional config
  HeightMapper(TransformLookup::Ptr tf, Config config = Config());

  ~HeightMapper() = default;

  // User interfaces
  void integrate(std::shared_ptr<PointCloud> cloud);
  HeightMap::ConstPtr getHeightMap() const;
  std::shared_ptr<const PointCloud> getScanProcessed() const;
  void reset();

  // Benchmarking interfaces
  void setBenchmarkEnabled(bool enable);
  void setBenchmarkInterval(size_t interval);
  flowpipe::PipelineProfiler *getProfiler() { return profiler_.get(); }

 private:
  void setupMappingPipeline();
  void setupDefaultPipeline();
  HeightMap::Ptr initMap(const Config &config);

  Config cfg_;
  TransformLookup::Ptr tf_tree_;
  HeightMap::Ptr map_;

  // Core pipeline components
  std::unique_ptr<flowpipe::Pipeline> mapping_pipeline_;
  std::unique_ptr<flowpipe::PipelineProfiler> profiler_;
  bool benchmark_enabled_ = false;

  mutable std::shared_mutex map_mutex_;
  mutable std::shared_mutex cloud_mutex_;
  std::shared_ptr<PointCloud> last_processed_cloud_;
};

}  // namespace height_mapping