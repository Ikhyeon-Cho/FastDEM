/*
 * FastHeightFilter.h
 *
 *  Created on: Dec 4, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_map_core/data/cloud_types.h"
#include <pcl/point_cloud.h>

namespace height_mapping {

class FastHeightFilter {
public:
  using Ptr = std::shared_ptr<FastHeightFilter>;

  FastHeightFilter(double minZ, double maxZ);

  template <typename PointT>
  void filter(const typename pcl::PointCloud<PointT>::Ptr &cloud,
              typename pcl::PointCloud<PointT>::Ptr &cloudFiltered);

private:
  // set min as infinite minus and max as infinite plus
  static constexpr double INF_MIN = -std::numeric_limits<double>::infinity();
  static constexpr double INF_MAX = std::numeric_limits<double>::infinity();
  double minZ_{INF_MIN}, maxZ_{INF_MAX};
};

} // namespace height_mapping
