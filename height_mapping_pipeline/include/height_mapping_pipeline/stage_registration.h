/*
 * stage_registration.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

// This file ensures all stage headers are included so their
// REGISTER_STAGE macros are executed during static initialization

#include "height_mapping_pipeline/stages/grid_ground_segmentation.h"
#include "height_mapping_pipeline/stages/height_estimation.h"
#include "height_mapping_pipeline/stages/move_origin.h"
#include "height_mapping_pipeline/stages/multi_sensor_sync.h"
#include "height_mapping_pipeline/stages/passthrough_filter.h"
#include "height_mapping_pipeline/stages/raycasting.h"
#include "height_mapping_pipeline/stages/statistical_ground_segmentation.h"
#include "height_mapping_pipeline/stages/transform_cloud.h"
#include "height_mapping_pipeline/stages/voxel_filter.h"