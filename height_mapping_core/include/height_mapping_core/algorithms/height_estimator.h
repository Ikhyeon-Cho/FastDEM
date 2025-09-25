/*
 * height_estimator.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ALGORITHMS_HEIGHT_ESTIMATOR_H
#define HEIGHT_MAPPING_CORE_ALGORITHMS_HEIGHT_ESTIMATOR_H

#include "height_mapping_core/data/height_map.h"
#include "height_mapping_core/data/point_cloud.h"
#include "height_mapping_core/estimators/height_estimator_base.h"
#include <map>
#include <memory>
#include <string>

namespace height_mapping::algorithms {

/**
 * @brief Height estimation algorithm for updating height map from point clouds
 *
 * Projects points to 2.5D grid and estimates height using configured estimator.
 * Maintains full feature set including sensor data layers and statistical
 * measures.
 */
class HeightEstimator {
public:
  /**
   * @brief Configuration parameters for height estimation
   */
  struct Config {
    enum class EstimatorType {
      INCREMENTAL_MEAN,
      KALMAN_FILTER,
      MOVING_AVERAGE
    };

    EstimatorType estimator_type = EstimatorType::INCREMENTAL_MEAN;

    // Kalman Filter parameters
    float kalman_process_noise = 0.01f;
    float kalman_measurement_noise = 0.1f;
    float kalman_initial_variance = 1.0f;

    // Moving Average parameters
    float moving_average_alpha = 0.3f;

    void validate() const {
      // Validate moving average alpha
      if (estimator_type == EstimatorType::MOVING_AVERAGE) {
        if (moving_average_alpha < 0.0f || moving_average_alpha > 1.0f) {
          throw std::invalid_argument("moving_average_alpha must be in [0, 1]");
        }
      }
      // Validate Kalman parameters
      if (estimator_type == EstimatorType::KALMAN_FILTER) {
        if (kalman_process_noise <= 0) {
          throw std::invalid_argument("kalman_process_noise must be positive");
        }
        if (kalman_measurement_noise <= 0) {
          throw std::invalid_argument(
              "kalman_measurement_noise must be positive");
        }
        if (kalman_initial_variance <= 0) {
          throw std::invalid_argument(
              "kalman_initial_variance must be positive");
        }
      }
    }
  };

  /**
   * @brief Estimation statistics
   */
  struct Stats {
    size_t points_processed = 0;
    size_t cells_updated = 0;
    size_t points_out_of_bounds = 0;
  };

private:
  Config config_;
  std::shared_ptr<estimators::HeightEstimatorBase> estimator_;

public:
  HeightEstimator();

  explicit HeightEstimator(const Config &config);

  void setConfig(const Config &config);

  const Config &getConfig() const { return config_; }

  /**
   * @brief Update height map with point cloud data
   * @param map Height map to update (modified)
   * @param cloud Point cloud with height measurements
   * @return Estimation statistics
   */
  Stats updateMap(HeightMap &map, const PointCloud &cloud) const;

private:
  void initializeSensorLayers(HeightMap &map, const PointCloud &cloud) const;

  void updateCell(HeightMap &map, const grid_map::Index &index,
                  const PointCloud::ConstPointView &point) const;

  void updateStatistics(HeightMap &map, const grid_map::Index &index,
                        float variance, float count) const;

  void updateSensorValue(float &current_value, float new_value,
                         float count) const;

  void updateSensorData(HeightMap &map, const grid_map::Index &index,
                        const PointCloud::ConstPointView &point,
                        float count) const;
};

} // namespace height_mapping::algorithms

#endif // HEIGHT_MAPPING_CORE_ALGORITHMS_HEIGHT_ESTIMATOR_H