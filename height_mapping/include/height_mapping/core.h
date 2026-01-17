/*
 * core.h
 *
 * Umbrella header for core types.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_H
#define HEIGHT_MAPPING_CORE_H

// Core data structures
#include "height_mapping/core/map.h"
#include "height_mapping/core/update.h"

// Height estimators
#include "height_mapping/core/estimator/base.h"
#include "height_mapping/core/estimator/incremental_mean.h"
#include "height_mapping/core/estimator/kalman_filter.h"
#include "height_mapping/core/estimator/moving_average.h"

// Layer names
#include "height_mapping/core/layers.h"

// Updaters
#include "height_mapping/core/updater/elevation.h"

// nanoPCL types
#include <nanopcl/core.hpp>
#include <nanopcl/transform.hpp>

namespace height_mapping {

// Re-export commonly used nanoPCL types
using npcl::Color;
using npcl::Point;
using npcl::PointCloud;
using npcl::Transformf;

}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_CORE_H
