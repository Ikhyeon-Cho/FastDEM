/*
 * IPointCloudProcessor.h
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

class IPointCloudProcessor {
public:
  using Ptr = std::shared_ptr<IPointCloudProcessor>;
  using ConstPtr = std::shared_ptr<const IPointCloudProcessor>;

  virtual ~IPointCloudProcessor() = default;

  // Rasterization - reduces point cloud to grid resolution
  template <typename PointT>
  virtual typename pcl::PointCloud<PointT>::Ptr
  rasterize(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, float grid_size) = 0;

  // Downsampling
  template <typename PointT>
  virtual typename pcl::PointCloud<PointT>::Ptr
  downsample(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, float leaf_size) = 0;

  // Range filtering
  template <typename PointT>
  virtual typename pcl::PointCloud<PointT>::Ptr
  filterByRange(const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                float min_range, float max_range) = 0;
};

} // namespace height_mapping