/*
 * stage_registry.cpp
 *
 * Centralized stage registration for all pipeline stages.
 * All stages are registered here via static initialization.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "flowpipe/stage_registration.h"

// Import all stage headers
#include "height_mapping/pipeline/stages/grid_ground_segmentation.h"
#include "height_mapping/pipeline/stages/height_estimation.h"
#include "height_mapping/pipeline/stages/move_origin.h"
#include "height_mapping/pipeline/stages/multi_sensor_sync.h"
#include "height_mapping/pipeline/stages/passthrough_filter.h"
#include "height_mapping/pipeline/stages/raycasting.h"
#include "height_mapping/pipeline/stages/statistical_ground_segmentation.h"
#include "height_mapping/pipeline/stages/transform_cloud.h"
#include "height_mapping/pipeline/stages/voxel_filter.h"

// Register all stages (executed before main())
namespace height_mapping::stages {

REGISTER_STAGE(GridGroundSegmentation)
REGISTER_STAGE(HeightEstimation)
REGISTER_STAGE(MoveOrigin)
REGISTER_STAGE(MultiSensorSync)
REGISTER_STAGE(PassthroughFilter)
REGISTER_STAGE(Raycasting)
REGISTER_STAGE(StatisticalGroundSegmentation)
REGISTER_STAGE(TransformCloud)
REGISTER_STAGE(VoxelFilter)

}  // namespace height_mapping::stages
