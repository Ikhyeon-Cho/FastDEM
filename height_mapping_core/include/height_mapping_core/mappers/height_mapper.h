/*
 * HeightMapper.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

// Height Map Core
#include "height_map_core/data/height_map.h"
#include "height_map_core/data/cloud_types.h"

// Height Mapping Algorithms
#include "height_mapping_core/estimators/stat_mean_estimator.h"
#include "height_mapping_core/estimators/kalman_estimator.h"
#include "height_mapping_core/estimators/moving_average_estimator.h"
#include "height_mapping_core/filters/fast_height_filter.h"
#include "height_mapping_core/correctors/height_map_raycaster.h"

namespace height_mapping {

class HeightMapper {
public:
  struct Config {
    std::string estimator_type;
    std::string frame_id;
    double map_length_x;
    double map_length_y;
    double grid_resolution;
    double min_height;
    double max_height;
  } cfg;

  HeightMapper(const Config &cfg);

  /**
   * @brief Gridded height mapping using height estimator
   * @param cloud: input pointcloud
   * @return filtered pointcloud
   */
  template <typename PointT>
  typename boost::shared_ptr<pcl::PointCloud<PointT>>
  heightMapping(const typename boost::shared_ptr<pcl::PointCloud<PointT>> &cloud);

  /*
   * Correct heightmap using raycasting
   * @param pointcloud: pointcloud for raycasting [ref: map frame]
   * @param sensorOrigin: sensor origin [ref: map frame]
   */
  template <typename PointT>
  void raycasting(const Eigen::Vector3f &sensorOrigin,
                  const typename boost::shared_ptr<pcl::PointCloud<PointT>> &cloud);

  /**
   * @brief Fast height filtering
   * @param cloud: input pointcloud
   * @param filtered_cloud: output filtered pointcloud
   */
  template <typename PointT>
  void
  fastHeightFilter(const typename boost::shared_ptr<pcl::PointCloud<PointT>> &cloud,
                   typename boost::shared_ptr<pcl::PointCloud<PointT>> &filtered_cloud);

  /**
   * @brief Move heightmap origin
   * @param position: new origin position in map frame
   */
  void moveMapOrigin(const grid_map::Position &position);

  /**
   * @brief Get heightmap
   * @return heightmap
   */
  const HeightMap &getHeightMap() const { return map_; }
  HeightMap &getHeightMap() { return map_; }

  /**
   * @brief Set heightmap origin
   * @param position: new origin position in map frame
   */
  void setMapPosition(const grid_map::Position &position) { map_.setPosition(position); }

  /**
   * @brief Clear heightmap
   */
  void clearMap() { map_.clearAll(); }

private:
  void initMap();
  void initHeightEstimator();

  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr
  cloudRasterization(const typename pcl::PointCloud<PointT>::Ptr &cloud, float gridSize);
  template <typename PointT>
  typename pcl::PointCloud<PointT>::Ptr
  cloudRasterizationAlt(const typename pcl::PointCloud<PointT>::Ptr &cloud,
                        float gridSize);

  struct pair_hash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const {
      auto h1 = std::hash<T1>{}(p.first);
      auto h2 = std::hash<T2>{}(p.second);
      return h1 ^ h2;
    }
  };

  // Members
  HeightMap map_;

  // Height mapping objects
  FastHeightFilter heightFilter_;
  HeightEstimatorBase::Ptr height_estimator_;
  HeightMapRaycaster raycaster_;
};
} // namespace height_mapping