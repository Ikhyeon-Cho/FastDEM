/*
 * grid_ground_segmentation.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_ALGORITHMS_GRID_GROUND_SEGMENTATION_H
#define HEIGHT_MAPPING_CORE_ALGORITHMS_GRID_GROUND_SEGMENTATION_H

#include <map>
#include <vector>

#include "height_mapping/core/data/point_cloud.h"

namespace height_mapping::algorithms {

/**
 * @brief Grid-based ground segmentation algorithm
 *
 * Segments point cloud into ground and non-ground points using grid-based
 * analysis. Each grid cell's ground level is estimated using percentile
 * statistics for robustness against outliers.
 */
class GridGroundSegmentation {
 public:
  /**
   * @brief Configuration parameters for grid-based segmentation
   */
  struct Config {
    float grid_resolution = 0.5f;    // Grid cell size in meters
    float cell_percentile = 0.2f;    // Percentile for robust minimum (0-1)
    float ground_thickness = 0.3f;   // Thickness of ground layer in meters
    float max_ground_height = 0.5f;  // Maximum height to consider as ground
    size_t min_points_per_cell = 2;  // Minimum points to process a cell
    bool keep_only_ground = true;    // Filter output to ground points only

    /**
     * @brief Validate configuration parameters
     * @throws std::invalid_argument if parameters are invalid
     */
    void validate() const {
      if (grid_resolution <= 0) {
        throw std::invalid_argument("grid_resolution must be positive");
      }
      if (cell_percentile < 0 || cell_percentile > 1) {
        throw std::invalid_argument("cell_percentile must be in [0, 1]");
      }
      if (ground_thickness <= 0) {
        throw std::invalid_argument("ground_thickness must be positive");
      }
      if (max_ground_height <= 0) {
        throw std::invalid_argument("max_ground_height must be positive");
      }
    }
  };

  /**
   * @brief Statistics from segmentation operation
   */
  struct Stats {
    size_t ground_count = 0;
    size_t obstacle_count = 0;
    size_t unprocessed_count = 0;
    size_t total_cells = 0;
    size_t processed_cells = 0;
    size_t obstacle_only_cells = 0;
  };

 private:
  Config config_;

  // Internal data structure for cell statistics
  struct CellData {
    std::vector<size_t> point_indices;
    std::vector<float> z_values;
    float robust_min_z = 0;
  };

 public:
  GridGroundSegmentation() : config_{} { config_.validate(); }

  /**
   * @brief Constructor with configuration
   * @param config Configuration parameters
   */
  explicit GridGroundSegmentation(const Config &config) : config_(config) {
    config_.validate();
  }

  /**
   * @brief Set new configuration
   * @param config New configuration parameters
   */
  void setConfig(const Config &config) {
    config_ = config;
    config_.validate();
  }

  /**
   * @brief Get current configuration
   * @return Current configuration
   */
  const Config &getConfig() const { return config_; }

  /**
   * @brief Segment point cloud in-place
   * @param cloud Point cloud to segment (modified in-place)
   * @return Statistics from the segmentation operation
   */
  Stats segmentInPlace(PointCloud &cloud) const;

  /**
   * @brief Segment copy of point cloud
   * @param cloud Input point cloud (not modified)
   * @param stats Statistics from the segmentation operation
   * @return Segmented copy of the cloud
   */
  PointCloud segment(const PointCloud &cloud, Stats &stats) const;

 private:
  /**
   * @brief Group points into grid cells
   * @param cloud Input point cloud
   * @return Map of cell coordinates to cell data
   */
  std::map<std::pair<int, int>, CellData> groupPointsByGrid(
      const PointCloud &cloud) const;

  /**
   * @brief Process cells to find robust ground level
   * @param cells_data Map of cells to process
   * @param[out] stats Statistics about processing
   */
  void processCells(std::map<std::pair<int, int>, CellData> &cells_data,
                    Stats &stats) const;

  /**
   * @brief Classify points based on cell ground levels
   * @param cloud Input cloud
   * @param cells_data Processed cell data
   * @return Filtered cloud based on configuration
   */
  PointCloud classifyAndFilter(
      const PointCloud &cloud,
      const std::map<std::pair<int, int>, CellData> &cells_data,
      Stats &stats) const;
};

}  // namespace height_mapping::algorithms

#endif  // HEIGHT_MAPPING_CORE_ALGORITHMS_GRID_GROUND_SEGMENTATION_H