/*
 * mapper.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_INTERFACES_MAPPER_H
#define HEIGHT_MAPPING_CORE_INTERFACES_MAPPER_H

#include "height_mapping/height_map.h"
#include "height_mapping/types.h"

namespace height_mapping {

/**
 * @brief Interface for height mapping implementations
 *
 * Defines the common API for mapping engines that integrate point clouds
 * into 2.5D elevation maps. Different implementations may use:
 * - Direct algorithm execution (HeightMapper)
 * - Configurable pipeline framework (ConfigurableHeightMapper)
 */
class IMapper {
 public:
  virtual ~IMapper() = default;

  /**
   * @brief Integrate point cloud into height map
   * @param cloud Point cloud to process (ownership shared)
   */
  virtual void integrate(std::shared_ptr<PointCloud> cloud) = 0;

  /**
   * @brief Get the current height map
   */
  virtual const HeightMap& getHeightMap() const = 0;

  /**
   * @brief Reset the height map to empty state
   */
  virtual void reset() = 0;

  /**
   * @brief Get the mapper implementation name
   * @return Human-readable name (e.g., "HeightMapper", "ConfigurableMapper")
   */
  virtual std::string name() const = 0;
};

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_CORE_INTERFACES_MAPPER_H
