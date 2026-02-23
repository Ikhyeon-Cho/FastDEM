// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#include "fastdem/mapping/elevation_mapping.hpp"

#include <cmath>
#include <grid_map_core/GridMapMath.hpp>

namespace fastdem {

ElevationMapping::ElevationMapping(ElevationMap& map,
                                   const config::Mapping& cfg)
    : map_(map), cfg_(cfg) {
  switch (cfg.estimation_type) {
    case EstimationType::P2Quantile: {
      const auto& p = cfg.p2;
      height_estimator_ = P2Quantile{p.dn0,
                                     p.dn1,
                                     p.dn2,
                                     p.dn3,
                                     p.dn4,
                                     p.elevation_marker,
                                     p.max_sample_count};
      break;
    }
    case EstimationType::Kalman: {
      const auto& kf = cfg.kalman;
      height_estimator_ =
          Kalman{kf.min_variance, kf.max_variance, kf.process_noise};
      break;
    }
  }

  // Create estimator layers (once)
  std::visit([&](auto& e) { e.ensureLayers(map_); }, height_estimator_);

  // Obstacle layer (always active, overwritten each frame)
  if (!map_.exists(layer::obstacle)) map_.add(layer::obstacle, NAN);
}

ElevationMapping::CellObservations ElevationMapping::rasterize(
    const PointCloud& cloud) {
  if (cloud.empty()) return {};

  const bool has_covariance = cloud.hasCovariance();
  const bool has_intensity = cloud.hasIntensity();
  const bool has_color = cloud.hasColor();

  CellObservations cells;
  cells.reserve(cloud.size());

  for (size_t i : cloud.indices()) {
    auto pt = cloud.point(i);
    grid_map::Index index;
    if (!map_.getIndex(grid_map::Position(pt.x(), pt.y()), index)) continue;

    float pt_z_var = 0.0f;
    if (has_covariance) {
      pt_z_var = cloud.covariance(i)(2, 2);
    }

    auto& cell = cells[index];
    const float z = pt.z();

    if (z < cell.min_z) {
      cell.min_z = z;
      cell.min_z_var = pt_z_var;
    }
    if (z > cell.max_z) {
      cell.max_z = z;
    }

    if (has_intensity) {
      float val = cloud.intensity(i);
      if (!cell.has_intensity || val > cell.max_intensity) {
        cell.max_intensity = val;
        cell.has_intensity = true;
      }
    }

    if (has_color) {
      auto color = cloud.color(i);
      Eigen::Vector3i rgb(color.r, color.g, color.b);
      float packed;
      grid_map::colorVectorToValue(rgb, packed);
      cell.color_packed = packed;
      cell.has_color = true;
    }
  }

  return cells;
}

void ElevationMapping::estimate(const CellObservations& observations) {
  if (observations.empty()) return;

  // Height estimator update (ground estimate - min_z per cell)
  std::visit(
      [&](auto& estimator) {
        estimator.bind(map_);

        for (const auto& [index, cell] : observations) {
          estimator.update(index, cell.min_z, cell.min_z_var);
          estimator.computeBounds(index);
        }
      },
      height_estimator_);
}

ElevationMapping::CellObservations ElevationMapping::update(
    const PointCloud& cloud, const Eigen::Vector2d& robot_position) {
  if (cfg_.mode == MappingMode::LOCAL) {
    map_.move(grid_map::Position(robot_position.x(), robot_position.y()));
  }

  auto obs = rasterize(cloud);
  if (obs.empty()) return obs;

  estimate(obs);
  updateMinMax(obs);
  updateObstacle(obs);
  if (cloud.hasIntensity()) updateIntensity(obs);
  if (cloud.hasColor()) updateColor(obs);
  return obs;
}

void ElevationMapping::updateMinMax(const CellObservations& obs) {
  auto& min_mat = map_.get(layer::elevation_min);
  auto& max_mat = map_.get(layer::elevation_max);
  for (const auto& [index, cell] : obs) {
    const int i = index(0);
    const int j = index(1);
    float& stored_min = min_mat(i, j);
    float& stored_max = max_mat(i, j);
    if (std::isnan(stored_min) || cell.min_z < stored_min) {
      stored_min = cell.min_z;
    }
    if (std::isnan(stored_max) || cell.max_z > stored_max) {
      stored_max = cell.max_z;
    }
  }
}

void ElevationMapping::updateObstacle(const CellObservations& obs) {
  auto& obstacle_mat = map_.get(layer::obstacle);
  map_.clear(layer::obstacle);
  for (const auto& [index, cell] : obs) {
    const int i = index(0);
    const int j = index(1);
    obstacle_mat(i, j) = (cell.max_z > cell.min_z) ? cell.max_z : NAN;
  }
}

void ElevationMapping::updateIntensity(const CellObservations& obs) {
  if (!map_.exists(layer::intensity)) map_.add(layer::intensity, NAN);
  auto& intensity_mat = map_.get(layer::intensity);
  for (const auto& [index, cell] : obs) {
    if (!cell.has_intensity) continue;
    const int i = index(0);
    const int j = index(1);
    float& stored = intensity_mat(i, j);
    if (std::isnan(stored) || cell.max_intensity > stored) {
      stored = cell.max_intensity;
    }
  }
}

void ElevationMapping::updateColor(const CellObservations& obs) {
  if (!map_.exists(layer::color)) map_.add(layer::color, NAN);
  auto& color_mat = map_.get(layer::color);
  for (const auto& [index, cell] : obs) {
    if (!cell.has_color) continue;
    color_mat(index(0), index(1)) = cell.color_packed;
  }
}

}  // namespace fastdem
