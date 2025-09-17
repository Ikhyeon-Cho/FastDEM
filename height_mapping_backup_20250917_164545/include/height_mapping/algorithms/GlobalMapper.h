/*
 * GlobalMapper.h
 *
 *  Created on: Dec 2, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_mapping/algorithms/HeightMapper.h"
#include <unordered_set>

// this is for the use of unordered_set with grid_map::Index
namespace std {
template <> struct hash<grid_map::Index> {
  std::size_t operator()(const grid_map::Index &index) const {
    std::size_t h1 = std::hash<int>{}(index[0]);
    std::size_t h2 = std::hash<int>{}(index[1]);
    return h1 ^ (h2 << 1);
  }
};

template <> struct equal_to<grid_map::Index> {
  bool operator()(const grid_map::Index &lhs, const grid_map::Index &rhs) const {
    return (lhs[0] == rhs[0]) && (lhs[1] == rhs[1]);
  }
};
} // namespace std

namespace height_mapping {

class GlobalMapper : public HeightMapper {
public:
  struct Config : public HeightMapper::Config {
    std::string map_save_dir;
  } cfg;

  GlobalMapper(const Config &cfg);

  template <typename PointT>
  typename boost::shared_ptr<pcl::PointCloud<PointT>>
  heightMapping(const typename boost::shared_ptr<pcl::PointCloud<PointT>> &cloud);

  const std::unordered_set<grid_map::Index> &getMeasuredGridIndices() const {
    return measured_indices_;
  }

private:
  template <typename PointT>
  void recordMeasuredCells(const height_mapping::HeightMap &map,
                           const pcl::PointCloud<PointT> &cloud);

  std::unordered_set<grid_map::Index> measured_indices_;
};
} // namespace height_mapping
