/*
 * elevation_point.h
 *
 *  Created on: Feb 10, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <pcl/point_types.h>

namespace height_mapping_types {

struct EIGEN_ALIGN16 ElevationPoint {
  PCL_ADD_POINT4D; // This adds the members x,y,z
  float elevation;
  float elevation_min;
  float elevation_max;
  float elevation_variance;
  float n_measurements;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace height_mapping_types

POINT_CLOUD_REGISTER_POINT_STRUCT(
    height_mapping_types::ElevationPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, elevation, elevation)(float,
                                                                         elevation_min,
                                                                         elevation_min)(
        float,
        elevation_max,
        elevation_max)(float, elevation_variance, elevation_variance)(float,
                                                                      n_measurements,
                                                                      n_measurements))
