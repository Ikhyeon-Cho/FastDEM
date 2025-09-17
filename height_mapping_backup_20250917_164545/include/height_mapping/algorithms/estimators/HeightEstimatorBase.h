/*
 * HeightEstimatorBase.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_mapping/algorithms/helper_functions.h"
#include "height_map_core/HeightMap.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace height_mapping {
class HeightEstimatorBase {
public:
  using Ptr = std::shared_ptr<HeightEstimatorBase>;

  virtual ~HeightEstimatorBase() = default;

  virtual void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZ> &cloud) = 0;
  virtual void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZI> &cloud) = 0;
  virtual void estimate(HeightMap &map,
                        const pcl::PointCloud<pcl::PointXYZRGB> &cloud) = 0;
};
} // namespace height_mapping
