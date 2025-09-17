/*
 * IHeightEstimator.h
 *
 *  Created on: Dec 17, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace height_mapping {

// Forward declaration
class HeightMap;

class IHeightEstimator {
public:
  using Ptr = std::shared_ptr<IHeightEstimator>;
  using ConstPtr = std::shared_ptr<const IHeightEstimator>;

  virtual ~IHeightEstimator() = default;

  // Pure virtual methods for height estimation
  virtual void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZ> &cloud) = 0;
  virtual void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZI> &cloud) = 0;
  virtual void estimate(HeightMap &map, const pcl::PointCloud<pcl::PointXYZRGB> &cloud) = 0;

  // Optional: Get estimator type name for debugging/logging
  virtual std::string getType() const = 0;
};

} // namespace height_mapping