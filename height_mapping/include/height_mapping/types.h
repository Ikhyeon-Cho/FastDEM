/*
 * types.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_TYPES_H
#define HEIGHT_MAPPING_TYPES_H

#include <nanopcl/nanopcl.hpp>

namespace height_mapping {

// Core types from nanoPCL
using Point = nanopcl::Point;
using PointCloud = nanopcl::PointCloud;
using Color = nanopcl::Color;

// Transform type
using Transformf = nanopcl::Transformf;

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_TYPES_H
