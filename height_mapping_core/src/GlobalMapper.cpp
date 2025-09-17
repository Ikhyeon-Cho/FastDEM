/*
 * GlobalMapper.cpp
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping_core/GlobalMapper.h"

namespace height_mapping {

GlobalMapper::GlobalMapper(const Config &cfg) : HeightMapper(cfg) {

  const auto &map = getHeightMap();
  measured_indices_.reserve(map.getSize().prod());
}

template <typename PointT>
typename boost::shared_ptr<pcl::PointCloud<PointT>> GlobalMapper::heightMapping(
    const typename boost::shared_ptr<pcl::PointCloud<PointT>> &cloud) {

  auto cloud_rasterized = HeightMapper::heightMapping<PointT>(cloud);

  // Save measured indices for efficiency
  recordMeasuredCells(getHeightMap(), *cloud_rasterized);

  return cloud_rasterized;
}

// Save measured indices for efficiency
template <typename PointT>
void GlobalMapper::recordMeasuredCells(const height_mapping::HeightMap &map,
                                       const pcl::PointCloud<PointT> &cloud) {

  grid_map::Index measured_cell_index;
  grid_map::Position measured_cell_position;

  for (const auto &point : cloud.points) {
    measured_cell_position.x() = point.x;
    measured_cell_position.y() = point.y;

    // Skip if the point is out of the map
    if (!map.getIndex(measured_cell_position, measured_cell_index))
      continue;

    if (map.isEmptyAt(measured_cell_index))
      continue;

    measured_indices_.insert(measured_cell_index);
  }
}

//////////////////////////////////////////////////
// Explicit instantiation of template functions //
//////////////////////////////////////////////////
// Laser
template typename boost::shared_ptr<pcl::PointCloud<Laser>>
GlobalMapper::heightMapping<Laser>(
    const typename boost::shared_ptr<pcl::PointCloud<Laser>> &cloud);
template void GlobalMapper::recordMeasuredCells(const height_mapping::HeightMap &map,
                                                const pcl::PointCloud<Laser> &cloud);

// Color
template typename boost::shared_ptr<pcl::PointCloud<Color>>
GlobalMapper::heightMapping<Color>(
    const typename boost::shared_ptr<pcl::PointCloud<Color>> &cloud);
template void GlobalMapper::recordMeasuredCells(const height_mapping::HeightMap &map,
                                                const pcl::PointCloud<Color> &cloud);

} // namespace height_mapping
