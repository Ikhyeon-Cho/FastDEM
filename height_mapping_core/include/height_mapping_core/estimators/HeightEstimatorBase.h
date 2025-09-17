/*
 * HeightEstimatorBase.h
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_mapping_core/interfaces/IHeightEstimator.h"
#include "height_mapping_core/helper_functions.h"
#include "height_map_core/HeightMap.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace height_mapping {

// HeightEstimatorBase now implements IHeightEstimator interface
// and provides common functionality for all estimators
class HeightEstimatorBase : public IHeightEstimator {
public:
  using Ptr = std::shared_ptr<HeightEstimatorBase>;

  virtual ~HeightEstimatorBase() = default;

  // Implement IHeightEstimator interface
  virtual void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZ> &cloud) override = 0;
  virtual void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZI> &cloud) override = 0;
  virtual void estimate(HeightMap &map,
                        const pcl::PointCloud<pcl::PointXYZRGB> &cloud) override = 0;

  // Default implementation for getType() - derived classes can override
  virtual std::string getType() const override { return "BaseEstimator"; }
};

} // namespace height_mapping
