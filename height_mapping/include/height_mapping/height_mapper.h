/*
 * height_mapper.h
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
#include <shared_mutex>

#include "height_mapping/interfaces/mapper.h"

namespace height_mapping {

// Forward declarations
struct HeightMapperConfig;
class IExtrinsicsProvider;
class IRobotPoseProvider;

namespace estimators {
class HeightEstimatorBase;
}

/**
 * @brief Height mapping engine with algorithm execution
 */
class HeightMapper : public IMapper {
 public:
  using Config = HeightMapperConfig;

  HeightMapper(const Config& config,
               std::shared_ptr<IExtrinsicsProvider> extrinsics,
               std::shared_ptr<IRobotPoseProvider> pose);
  ~HeightMapper();

  // Non-copyable
  HeightMapper(const HeightMapper&) = delete;
  HeightMapper& operator=(const HeightMapper&) = delete;

  void integrate(std::shared_ptr<PointCloud> cloud) override;
  const HeightMap& getHeightMap() const override;
  void reset() override;
  std::string name() const override { return "HeightMapper"; }

 private:
  void initializeHeightEstimator();

  std::shared_ptr<IExtrinsicsProvider> extrinsics_;
  std::shared_ptr<IRobotPoseProvider> pose_;
  std::unique_ptr<Config> config_;
  std::unique_ptr<HeightMap> map_;
  std::unique_ptr<estimators::HeightEstimatorBase> height_estimator_;

  mutable std::shared_mutex map_mutex_;
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_HEIGHT_MAPPER_H
