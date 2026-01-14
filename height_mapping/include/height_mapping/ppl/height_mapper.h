/*
 * height_mapper.h
 *
 * Pipeline-based HeightMapper using ppl (Pipeline Library).
 * For experimental/research use with YAML-configurable pipeline.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PPL_HEIGHT_MAPPER_H
#define HEIGHT_MAPPING_PPL_HEIGHT_MAPPER_H

#include <ppl/ppl.h>

#include <memory>
#include <shared_mutex>
#include <string>

#include "height_mapping/interfaces/mapper.h"
#include "height_mapping/ppl/types.h"

namespace height_mapping {

// Forward declarations
class IExtrinsicsProvider;
class IRobotPoseProvider;

namespace ppl {

/**
 * @brief Pipeline-based height mapper using ppl library
 */
class HeightMapper : public IMapper {
 public:
  struct Config {
    HeightMap::Config map;
    YAML::Node pipeline;  // Cached YAML for pipeline loading
    bool enable_debug{false};

    // Profiling options
    bool enable_profiling{false};
    size_t profile_interval{100};

    static Config load(const std::string& path);
  };

  HeightMapper(const Config& config,
               std::shared_ptr<IExtrinsicsProvider> extrinsics,
               std::shared_ptr<IRobotPoseProvider> pose);
  ~HeightMapper();

  // Non-copyable
  HeightMapper(const HeightMapper&) = delete;
  HeightMapper& operator=(const HeightMapper&) = delete;

  // Core API
  void integrate(std::shared_ptr<PointCloud> cloud) override;
  const HeightMap& getHeightMap() const override;
  void reset() override;
  std::string name() const override { return "ppl::HeightMapper"; }

 private:
  void loadPipeline(const YAML::Node& yaml);

  std::shared_ptr<IExtrinsicsProvider> extrinsics_;
  std::shared_ptr<IRobotPoseProvider> pose_;
  std::shared_ptr<HeightMap> map_;
  MappingPipeline pipeline_;
  std::unique_ptr<MappingProfiler> profiler_;

  mutable std::shared_mutex map_mutex_;
};

}  // namespace ppl
}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_PPL_HEIGHT_MAPPER_H
