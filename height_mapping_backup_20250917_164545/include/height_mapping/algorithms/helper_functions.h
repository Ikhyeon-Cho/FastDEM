/*
 * helper_functions.h
 *
 *  Created on: Nov 25, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <iostream>
#include <pcl/point_cloud.h>

template <typename PointT> bool hasEmptyCloud(const pcl::PointCloud<PointT> &cloud) {
  if (cloud.empty()) {
    std::cout << "\033[33m[HeightEstimator]: Input cloud is empty! \033[0m" << std::endl;
    return true;
  }
  return false;
}
