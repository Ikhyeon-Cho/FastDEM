/*
 * height_mapper.h
 *
 * Pipeline-based HeightMapper using ppl (Pipeline Library).
 * For experimental/research use with YAML-configurable pipeline.
 *
 * For standard use with direct algorithm execution,
 * see height_mapping::HeightMapper.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PPL_HEIGHT_MAPPER_H
#define HEIGHT_MAPPING_PPL_HEIGHT_MAPPER_H

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>

#include "height_mapping/height_map.h"
#include "height_mapping/interfaces/extrinsics_provider.h"
#include "height_mapping/interfaces/mapper.h"
#include "height_mapping/interfaces/robot_pose_provider.h"
#include "height_mapping/ppl/frame.h"
#include "height_mapping/ppl/types.h"
#include "height_mapping/types.h"

namespace height_mapping::ppl {

/**
 * @brief Pipeline-based height mapper using ppl library
 *
 * Uses ppl (Pipeline Library) for YAML-based pipeline configuration.
 */
class HeightMapper : public IMapper {
 public:
  /**
   * @brief Configuration for ppl::HeightMapper
   */
  struct Config {
    HeightMap::Config map;
    bool enable_debug{false};

    // Path to pipeline YAML configuration file (ppl format)
    std::string pipeline_config_path;

    /// Load configuration from YAML file
    static Config load(const std::string& path);
  };

  /**
   * @brief Construct a fully initialized HeightMapper
   *
   * @param config Complete mapping configuration (loaded externally)
   * @param extrinsics Provider for sensor extrinsics (Sensor -> Base)
   * @param pose Provider for robot pose (Base -> Map)
   *
   * @throws std::runtime_error if pipeline construction fails
   */
  HeightMapper(const Config& config, IExtrinsicsProvider::Ptr extrinsics,
               IRobotPoseProvider::Ptr pose);

  ~HeightMapper() = default;

  // Core API (IMapper)
  void integrate(std::shared_ptr<PointCloud> cloud) override;
  const HeightMap& getHeightMap() const override;
  void reset() override;
  std::string name() const override { return "ppl::HeightMapper"; }

 private:
  Config config_;

  // Dependencies (injected)
  IExtrinsicsProvider::Ptr extrinsics_;
  IRobotPoseProvider::Ptr pose_provider_;

  // Output
  std::shared_ptr<HeightMap> map_;

  // Pipeline loaded from YAML
  ::ppl::Pipeline<MappingFrame> pipeline_;

  mutable std::shared_mutex map_mutex_;
};

}  // namespace height_mapping::ppl

#endif  // HEIGHT_MAPPING_PPL_HEIGHT_MAPPER_H
