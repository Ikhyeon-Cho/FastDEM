#include "height_mapping/segmenters/grid_ground_segmenter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

namespace height_mapping::segmenters {

namespace {

struct CellData {
  std::vector<size_t> point_indices;
  std::vector<float> z_values;
  float robust_min_z = 0;
  bool is_obstacle_only = false;
};

using CellMap = std::map<std::pair<int, int>, CellData>;

CellMap groupPointsByGrid(const PointCloud& cloud, float resolution) {
  CellMap cells;

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& pt = cloud[i];
    int gx = static_cast<int>(std::floor(pt.x() / resolution));
    int gy = static_cast<int>(std::floor(pt.y() / resolution));

    auto& cell = cells[{gx, gy}];
    cell.point_indices.push_back(i);
    cell.z_values.push_back(pt.z());
  }

  return cells;
}

void processCells(CellMap& cells,
                  const GridGroundSegmenter::Parameters& params) {
  for (auto& [key, cell] : cells) {
    if (cell.z_values.size() < params.min_points_per_cell) {
      cell.is_obstacle_only = true;
      continue;
    }

    // Sort for percentile
    std::sort(cell.z_values.begin(), cell.z_values.end());

    size_t idx =
        static_cast<size_t>(params.cell_percentile * (cell.z_values.size() - 1));
    idx = std::min(idx, cell.z_values.size() - 1);
    cell.robust_min_z = cell.z_values[idx];

    // Check if cell is too high to be ground
    if (cell.robust_min_z > params.max_ground_height) {
      cell.is_obstacle_only = true;
    }
  }
}

}  // namespace

GridGroundSegmenter::GridGroundSegmenter(const Parameters& params)
    : params_(params) {}

void GridGroundSegmenter::segment(const PointCloud& input, PointCloud& ground,
                                  PointCloud& obstacles) {
  ground.clear();
  obstacles.clear();

  if (input.empty()) {
    return;
  }

  // Group and process cells
  auto cells = groupPointsByGrid(input, params_.grid_resolution);
  processCells(cells, params_);

  // Prepare output clouds
  const bool has_intensity = input.hasIntensity();
  const bool has_color = input.hasColor();

  if (has_intensity) {
    ground.enableIntensity();
    obstacles.enableIntensity();
  }
  if (has_color) {
    ground.enableColor();
    obstacles.enableColor();
  }

  ground.reserve(input.size() / 2);
  obstacles.reserve(input.size() / 2);

  // Classify points
  for (size_t i = 0; i < input.size(); ++i) {
    const auto& pt = input[i];
    int gx = static_cast<int>(std::floor(pt.x() / params_.grid_resolution));
    int gy = static_cast<int>(std::floor(pt.y() / params_.grid_resolution));

    auto it = cells.find({gx, gy});
    if (it == cells.end()) {
      continue;
    }

    const auto& cell = it->second;

    // Obstacle-only cell or above ground threshold
    bool is_obstacle =
        cell.is_obstacle_only ||
        (pt.z() > cell.robust_min_z + params_.ground_thickness);

    if (is_obstacle) {
      obstacles.xyz().push_back(pt);
      if (has_intensity) obstacles.intensity().push_back(input.intensity()[i]);
      if (has_color) obstacles.color().push_back(input.color()[i]);
    } else {
      ground.xyz().push_back(pt);
      if (has_intensity) ground.intensity().push_back(input.intensity()[i]);
      if (has_color) ground.color().push_back(input.color()[i]);
    }
  }

  // Preserve metadata
  ground.setTimestamp(input.timestamp());
  ground.setFrameId(input.frameId());
  obstacles.setTimestamp(input.timestamp());
  obstacles.setFrameId(input.frameId());
}

}  // namespace height_mapping::segmenters
