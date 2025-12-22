/*
 * height_mapper.h
 *
 * Standard HeightMapper - Direct algorithm calls without Pipeline.
 *
 * This is the recommended mapper for standard use:
 * - Simple and predictable execution flow
 * - No Pipeline framework dependency
 * - Easy to understand and debug
 *
 * For experimental/research use with configurable pipeline,
 * see ppl::HeightMapper.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_HEIGHT_MAPPER_H
#define HEIGHT_MAPPING_HEIGHT_MAPPER_H

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>

// nanoPCL
#include <nanopcl/filters/passthrough.hpp>
#include <nanopcl/filters/voxel_grid.hpp>
#include <nanopcl/transform/operations.hpp>

// Core algorithms
#include "height_mapping/algorithms/algorithms.h"
#include "height_mapping/estimators/kalman_filter.h"
#include "height_mapping/estimators/moving_average.h"
#include "height_mapping/height_map.h"
#include "height_mapping/interfaces/extrinsics_provider.h"
#include "height_mapping/interfaces/mapper.h"
#include "height_mapping/interfaces/robot_pose_provider.h"
#include "height_mapping/types.h"

namespace height_mapping {

/// Mapping mode: robot-centric (local) or map-centric (global)
enum class MappingMode { ROBOT_CENTRIC, MAP_CENTRIC };

/**
 * @brief Standard height mapper with algorithm execution
 */
class HeightMapper : public IMapper {
 public:
  /**
   * @brief Configuration for HeightMapper
   */
  struct Config {
    HeightMap::Config map;
    MappingMode mode{MappingMode::ROBOT_CENTRIC};

    // Algorithm Configs
    nanopcl::filters::VoxelGrid::Config voxel_filter;
    nanopcl::filters::Passthrough::Config passthrough_filter;

    struct {
      std::string method{"grid"};  // grid, statistical
      algorithms::GridGroundSegmentationConfig grid;
      algorithms::StatisticalGroundSegmentationConfig statistical;
    } ground_segmentation;

    algorithms::RaycastingConfig raycasting;

    struct {
      std::string type{"incremental_mean"};
      estimators::KalmanFilter::Parameters kalman;
      estimators::MovingAverage::Parameters moving_average;
    } height_estimation;

    /// Load configuration from YAML file
    static Config load(const std::string& path);
  };

  /**
   * @brief Construct HeightMapper with configuration and providers
   *
   * @param config Configuration (loaded from YAML or constructed)
   * @param extrinsics Sensor extrinsics provider (Sensor -> Base)
   * @param pose Robot pose provider (Base -> Map)
   */
  HeightMapper(const Config& config, IExtrinsicsProvider::Ptr extrinsics,
               IRobotPoseProvider::Ptr pose);

  ~HeightMapper() = default;

  // Non-copyable
  HeightMapper(const HeightMapper&) = delete;
  HeightMapper& operator=(const HeightMapper&) = delete;

  /**
   * @brief Integrate point cloud into height map
   * @param cloud Point cloud to process (ownership shared, content modified)
   */
  void integrate(std::shared_ptr<PointCloud> cloud) override;

  /**
   * @brief Get the current height map
   */
  const HeightMap& getHeightMap() const override;

  /**
   * @brief Reset the height map
   */
  void reset() override;

  /**
   * @brief Get the mapper name
   */
  std::string name() const override { return "HeightMapper"; }

 private:
  void initializeAlgorithms();

  Config config_;

  // Providers
  IExtrinsicsProvider::Ptr extrinsics_;
  IRobotPoseProvider::Ptr pose_;

  // Output
  std::unique_ptr<HeightMap> map_;

  // Point cloud filters
  nanopcl::filters::VoxelGrid voxel_filter_;
  nanopcl::filters::Passthrough passthrough_filter_;

  // Height estimator
  std::unique_ptr<estimators::HeightEstimatorBase> height_estimator_;

  // Thread safety
  mutable std::shared_mutex map_mutex_;
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_HEIGHT_MAPPER_H
