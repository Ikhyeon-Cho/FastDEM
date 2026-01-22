/*
 * mapper.h
 *
 * Pipeline-based Mapper using ppl (Pipeline Library).
 * For experimental/research use with YAML-configurable pipeline.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_LAB_MAPPER_H
#define HEIGHT_MAPPING_LAB_MAPPER_H

#include <ppl/ppl.h>

#include <memory>
#include <shared_mutex>
#include <string>

#include "height_mapping/config/map.h"
#include "height_mapping/lab/frame.h"

namespace height_mapping {

// Forward declarations
class IExtrinsicsProvider;
class IRobotPoseProvider;

namespace lab {

/**
 * @brief Pipeline-based height mapper using ppl library
 */
class Mapper {
 public:
  struct Config {
    config::Map map;
    YAML::Node pipeline;  // Cached YAML for pipeline loading
    bool enable_debug{false};

    // Profiling options
    bool enable_profiling{false};
    size_t profile_interval{100};

    static Config load(const std::string& path);
  };

  Mapper(const Config& config, std::shared_ptr<IExtrinsicsProvider> extrinsics,
         std::shared_ptr<IRobotPoseProvider> pose);
  ~Mapper();

  // Non-copyable
  Mapper(const Mapper&) = delete;
  Mapper& operator=(const Mapper&) = delete;

  // Core API
  void update(std::shared_ptr<PointCloud> cloud);
  const HeightMap& map() const;
  void reset();

 private:
  void loadPipeline(const YAML::Node& yaml);

  std::shared_ptr<IExtrinsicsProvider> extrinsics_;
  std::shared_ptr<IRobotPoseProvider> pose_;
  std::shared_ptr<HeightMap> map_;
  MappingPipeline pipeline_;
  std::unique_ptr<MappingProfiler> profiler_;

  mutable std::shared_mutex map_mutex_;
};

}  // namespace lab
}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_LAB_MAPPER_H
