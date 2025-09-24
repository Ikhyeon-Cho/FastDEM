#pragma once

#include "height_mapping_core/config.h"
#include "height_mapping_core/data/height_map.h"
#include "height_mapping_core/geometry/point_cloud.h"
#include "height_mapping_core/interfaces/transform_provider.h"
#include "height_mapping_core/pipeline/mapping_context.h"
#include "pipeline_core/pipeline.h"
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>

namespace height_mapping::core {
using PointCloud = geometry::PointCloud;

class MappingEngine {
public:
  // Main constructor with config structure
  MappingEngine(std::shared_ptr<ITransformProvider> transform_provider,
                const Config &config);

  // Loads from file
  MappingEngine(std::shared_ptr<ITransformProvider> transform_provider,
                const std::string &config_file = "")
      : MappingEngine(transform_provider, Config::fromFile(config_file)) {
  }

  ~MappingEngine() = default;

  // User interfaces
  void registerCloud(std::shared_ptr<PointCloud> cloud);
  std::shared_ptr<const height_map::HeightMap> getHeightMap() const;
  std::shared_ptr<const PointCloud> getProcessedCloud() const;
  void reset();

private:
  void setupMappingPipeline();
  void setupDefaultPipeline();
  void initializeMap();

  Config config_;
  std::shared_ptr<ITransformProvider> transform_provider_;

  std::shared_ptr<height_map::HeightMap> map_;
  std::unique_ptr<pipeline::Pipeline> mapping_pipeline_;

  mutable std::shared_mutex map_mutex_;
  mutable std::shared_mutex cloud_mutex_;
  std::shared_ptr<PointCloud> last_processed_cloud_;
};

} // namespace height_mapping::core