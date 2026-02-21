// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef FASTDEM_MAPPING_ELEVATION_MAPPING_HPP
#define FASTDEM_MAPPING_ELEVATION_MAPPING_HPP

#include <limits>
#include <memory>
#include <variant>

#include "fastdem/config/mapping.hpp"
#include "fastdem/elevation_map.hpp"
#include "fastdem/mapping/grid_index_hash.hpp"
#include "fastdem/mapping/kalman_estimation.hpp"
#include "fastdem/mapping/quantile_estimation.hpp"
#include "fastdem/point_types.hpp"

namespace fastdem {

/// Scan-sequential elevation mapping.
/// Updates map layers via temporal height estimation (Kalman or P² quantile).
class ElevationMapping {
 public:
  using HeightEstimator = std::variant<Kalman, P2Quantile>;

  /// Per-cell observation from a single scan.
  struct CellObservation {
    float min_z = std::numeric_limits<float>::max();
    float min_z_var = 0.0f;
    float max_z = std::numeric_limits<float>::lowest();
    float max_intensity = std::numeric_limits<float>::lowest();
    float color_packed = 0.0f;
    bool has_intensity = false;
    bool has_color = false;
  };

  using CellObservations = CellMap<CellObservation>;

  ElevationMapping(ElevationMap& map, const config::Mapping& cfg);

  /// Main api: rasterize + estimate in one call. Returns cell observations.
  CellObservations update(const PointCloud& cloud,
                          const Eigen::Vector2d& robot_position);

  /// Bin points into per-cell observations.
  CellObservations rasterize(const PointCloud& cloud);

  /// Apply height estimation to map layers (elevation, variance, bounds).
  void estimate(const CellObservations& obs);

 private:
  void updateMinMax(const CellObservations& obs);
  void updateObstacle(const CellObservations& obs);
  void updateIntensity(const CellObservations& obs);
  void updateColor(const CellObservations& obs);

  ElevationMap& map_;
  config::Mapping cfg_;
  HeightEstimator height_estimator_;
};

inline std::unique_ptr<ElevationMapping> createElevationMapping(
    ElevationMap& map, const config::Mapping& cfg) {
  return std::make_unique<ElevationMapping>(map, cfg);
}

}  // namespace fastdem

#endif  // FASTDEM_MAPPING_ELEVATION_MAPPING_HPP
