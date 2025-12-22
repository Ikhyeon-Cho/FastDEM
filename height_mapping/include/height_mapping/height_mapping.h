/*
 * height_mapping.h
 *
 * Single-include facade header for height_mapping library.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_H
#define HEIGHT_MAPPING_H

// Core Types
#include "height_mapping/types.h"
#include "height_mapping/height_map.h"

// Interfaces
#include "height_mapping/interfaces/mapper.h"
#include "height_mapping/interfaces/extrinsics_provider.h"
#include "height_mapping/interfaces/robot_pose_provider.h"

// Mappers
#include "height_mapping/height_mapper.h"
#include "height_mapping/ppl/height_mapper.h"

#endif  // HEIGHT_MAPPING_H
