#pragma once

#include "height_mapping/segmenters/ground_segmenter_base.h"

namespace height_mapping::segmenters {

/**
 * @brief Ground segmentation using grid-based cell analysis.
 *
 * Divides the point cloud into 2D grid cells and analyzes each cell:
 * - Computes robust minimum per cell using percentile
 * - Ground: points within [cell_min, cell_min + thickness]
 * - Obstacles: points above threshold or in obstacle-only cells
 */
class GridGroundSegmenter : public GroundSegmenterBase {
 public:
  struct Parameters {
    float grid_resolution = 0.5f;    // Grid cell size [m]
    float cell_percentile = 0.2f;    // Percentile for robust minimum (0-1)
    float ground_thickness = 0.3f;   // Ground layer thickness [m]
    float max_ground_height = 0.5f;  // Max height to consider as ground [m]
    size_t min_points_per_cell = 2;  // Min points to process a cell
  };

  GridGroundSegmenter() = default;
  explicit GridGroundSegmenter(const Parameters& params);

  void segment(const PointCloud& input, PointCloud& ground,
               PointCloud& obstacles) override;

  std::string name() const override { return "GridGroundSegmenter"; }

 private:
  Parameters params_;
};

}  // namespace height_mapping::segmenters
