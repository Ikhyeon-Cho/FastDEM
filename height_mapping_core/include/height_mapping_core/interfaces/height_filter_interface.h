/*
 * IHeightFilter.h
 *
 *  Created on: Dec 17, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <memory>
#include <pcl/point_cloud.h>
#include <boost/shared_ptr.hpp>

namespace height_mapping {

class IHeightFilter {
public:
  using Ptr = std::shared_ptr<IHeightFilter>;
  using ConstPtr = std::shared_ptr<const IHeightFilter>;

  virtual ~IHeightFilter() = default;

  // Filter point cloud by height thresholds
  template <typename PointT>
  virtual void filter(const typename boost::shared_ptr<pcl::PointCloud<PointT>> &cloud,
                      typename boost::shared_ptr<pcl::PointCloud<PointT>> &filtered_cloud) = 0;

  // Set height thresholds
  virtual void setMinHeight(float min_height) = 0;
  virtual void setMaxHeight(float max_height) = 0;

  // Get current thresholds
  virtual float getMinHeight() const = 0;
  virtual float getMaxHeight() const = 0;
};

} // namespace height_mapping