// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Bridge module: PCL pcl::PointCloud<T> conversion.
//
// Supported PCL point types:
//   - PointXYZ, PointXYZI, PointXYZL
//   - PointXYZRGB, PointXYZRGBA, PointXYZRGBL
//   - PointXYZINormal, PointXYZRGBNormal
//   - PointNormal, PointXYZLNormal
//   - Any custom type with x, y, z members
//
// Usage:
//   #include <nanopcl/bridge/pcl.hpp>
//
//   pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud = ...;
//   auto cloud = npcl::from(*pcl_cloud);
//   auto back = npcl::to<pcl::PointCloud<pcl::PointXYZI>>(cloud);

#ifndef NANOPCL_BRIDGE_PCL_HPP
#define NANOPCL_BRIDGE_PCL_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "nanopcl/bridge/convert.hpp"

namespace npcl {

// =============================================================================
// PCL PointCloud Conversion (via Convert<> specialization)
// =============================================================================

/**
 * @brief Convert specialization for PCL PointCloud
 *
 * Automatically detects and converts available fields based on PCL point type.
 * Enables:
 *   auto cloud = npcl::from(pcl_cloud);
 *   auto pcl_cloud = npcl::to<pcl::PointCloud<pcl::PointXYZI>>(cloud);
 *
 * @tparam PointT PCL point type (e.g., pcl::PointXYZI)
 */
template <typename PointT>
struct Convert<pcl::PointCloud<PointT>>;

// =============================================================================
// Convenience Functions for PCL Smart Pointers
// =============================================================================

/**
 * @brief Convert PCL PointCloud::Ptr to nanoPCL PointCloud
 *
 * @tparam PointT PCL point type
 * @param pcl_cloud PCL shared pointer
 * @return PointCloud converted cloud with detected channels
 */
template <typename PointT>
[[nodiscard]] PointCloud from(
    const typename pcl::PointCloud<PointT>::Ptr& pcl_cloud);

/**
 * @brief Convert PCL PointCloud::ConstPtr to nanoPCL PointCloud
 *
 * @tparam PointT PCL point type
 * @param pcl_cloud PCL const shared pointer
 * @return PointCloud converted cloud with detected channels
 */
template <typename PointT>
[[nodiscard]] PointCloud from(
    const typename pcl::PointCloud<PointT>::ConstPtr& pcl_cloud);

/**
 * @brief Convert nanoPCL PointCloud to PCL PointCloud::Ptr
 *
 * @tparam PointT Target PCL point type
 * @param cloud nanoPCL point cloud
 * @return pcl::PointCloud<PointT>::Ptr shared pointer to converted cloud
 *
 * @note Only channels supported by PointT are written. Other channels in
 *       the input cloud are silently ignored.
 *
 * @code
 * auto pcl_cloud = npcl::toPclPtr<pcl::PointXYZI>(cloud);
 * @endcode
 */
template <typename PointT>
[[nodiscard]] typename pcl::PointCloud<PointT>::Ptr toPclPtr(
    const PointCloud& cloud);

}  // namespace npcl

#include "nanopcl/bridge/impl/pcl_impl.hpp"

#endif  // NANOPCL_BRIDGE_PCL_HPP
