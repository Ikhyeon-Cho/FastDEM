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

#include <nanopcl/core.hpp>
#include <nanopcl/transform.hpp>

#include "height_map/height_map.h"

namespace height_mapping {

// nanoPCL types
using npcl::Point;
using npcl::PointCloud;
using npcl::Color;
using npcl::Transformf;

// height_map types
using height_map::HeightMap;
namespace layer = height_map::layer;

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_TYPES_H
