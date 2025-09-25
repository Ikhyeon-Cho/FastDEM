/*
 * grid_ground_segmentation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_STAGES_GRID_GROUND_SEGMENTATION_H
#define HEIGHT_MAPPING_CORE_STAGES_GRID_GROUND_SEGMENTATION_H

#include "pipeline_core/stage.h"
#include <map>
#include <vector>

namespace height_mapping::core::stages {

/**
 * @brief Grid-based ground segmentation using cell-wise minimum approach
 *
 * This stage uses a grid-based method to extract ground points:
 * 1. Divides space into grid cells
 * 2. Finds minimum z-value per cell (with percentile for robustness)
 * 3. Defines ground as points within thickness from cell minimum
 * 4. Handles each cell independently for local terrain adaptation
 */
class GridGroundSegmentation : public pipeline::Stage {
public:
  GridGroundSegmentation();

  /**
   * @brief Configure the stage from parameters
   * @param params Map of parameter name to value strings
   *  - "grid_resolution": Size of grid cells in meters (default: 0.5m)
   *  - "cell_percentile": Percentile for robust cell minimum (default: 0.2 = 20th)
   *  - "ground_thickness": Thickness of ground layer from cell min (default: 0.3m)
   *  - "max_ground_height": Maximum height to consider as ground (default: 0.5m)
   *  - "min_points_per_cell": Minimum points to process a cell (default: 2)
   *  - "keep_only_ground": If true, keeps only ground points (default: true)
   */
  void configure(const std::map<std::string, std::string> &params) override;

protected:
  void processImpl(pipeline::Context &ctx) override;

private:
  struct CellData {
    std::vector<size_t> point_indices;
    std::vector<float> z_values;
    float robust_min_z = 0.0f;
  };

  // Configuration parameters
  float grid_resolution_ = 0.5f;     // Grid cell size (50cm default)
  float cell_percentile_ = 0.2f;     // 20th percentile for robust cell minimum
  float ground_thickness_ = 0.3f;    // Thickness of ground layer
  float max_ground_height_ = 0.5f;   // Maximum height to consider as ground (in base_link)
  size_t min_points_per_cell_ = 2;   // Minimum points to process cell
  bool keep_only_ground_ = true;     // Filter to ground points only
};

} // namespace height_mapping::core::stages

#endif // HEIGHT_MAPPING_CORE_STAGES_GRID_GROUND_SEGMENTATION_H