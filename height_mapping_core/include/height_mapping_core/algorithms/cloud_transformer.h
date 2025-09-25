/*
 * cloud_transformer.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ALGORITHMS_CLOUD_TRANSFORMER_H
#define HEIGHT_MAPPING_CORE_ALGORITHMS_CLOUD_TRANSFORMER_H

#include "height_mapping_core/data/point_cloud.h"
#include "height_mapping_core/data/transform.h"
#include <string>

namespace height_mapping::algorithms {

/**
 * @brief Point cloud transformation algorithm
 *
 * Transforms point clouds between coordinate frames using provided transform.
 */
class CloudTransformer {
public:
  /**
   * @brief Configuration parameters for cloud transformer
   */
  struct Config {
    std::string target_frame = "base_link";
    bool skip_if_same_frame = true;

    void validate() const {
      if (target_frame.empty()) {
        throw std::invalid_argument("target_frame cannot be empty");
      }
    }
  };

  /**
   * @brief Transform statistics
   */
  struct Stats {
    std::string original_frame;
    std::string target_frame;
    bool transform_applied = false;
    size_t points_transformed = 0;
  };

private:
  Config config_;

public:
  CloudTransformer() : config_{} { config_.validate(); }

  explicit CloudTransformer(const Config &config) : config_(config) {
    config_.validate();
  }

  void setConfig(const Config &config) {
    config_ = config;
    config_.validate();
  }

  const Config &getConfig() const { return config_; }

  /**
   * @brief Transform point cloud in-place
   * @param cloud Point cloud to transform (modified)
   * @param transform Transform to apply
   * @return Transform statistics
   */
  Stats transformInPlace(PointCloud &cloud, const Transform3D &transform) const;

  /**
   * @brief Transform point cloud (preserves original)
   * @param cloud Input point cloud (preserved)
   * @param transform Transform to apply
   * @param stats Output statistics
   * @return New transformed cloud
   */
  PointCloud transform(const PointCloud &cloud, const Transform3D &transform,
                       Stats &stats) const;

  /**
   * @brief Check if transformation is needed
   * @param cloud Input point cloud
   * @return true if cloud needs transformation
   */
  bool needsTransform(const PointCloud &cloud) const;
};

} // namespace height_mapping::algorithms

#endif // HEIGHT_MAPPING_CORE_ALGORITHMS_CLOUD_TRANSFORMER_H