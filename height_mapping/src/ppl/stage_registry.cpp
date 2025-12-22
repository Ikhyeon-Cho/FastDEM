/*
 * stage_registry.cpp
 *
 * Explicit stage registration for height mapping pipeline.
 * Uses PPL_REGISTER_STAGE macros + explicit function to prevent linker stripping.
 *
 *  Created on: Jan 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/ppl/stage_registry.h"

#include <ppl/ppl.h>

// Import all stage headers
#include "height_mapping/ppl/frame.h"
#include "height_mapping/ppl/stages/grid_ground_segmentation.h"
#include "height_mapping/ppl/stages/height_estimation.h"
#include "height_mapping/ppl/stages/move_origin.h"
#include "height_mapping/ppl/stages/multi_sensor_sync.h"
#include "height_mapping/ppl/stages/passthrough_filter.h"
#include "height_mapping/ppl/stages/raycasting.h"
#include "height_mapping/ppl/stages/statistical_ground_segmentation.h"
#include "height_mapping/ppl/stages/voxel_filter.h"

// Bring types into scope to avoid namespace in macro expansion
using MappingFrame = height_mapping::ppl::MappingFrame;
using VoxelFilter = height_mapping::ppl::stages::VoxelFilter;
using PassthroughFilter = height_mapping::ppl::stages::PassthroughFilter;
using GridGroundSegmentation = height_mapping::ppl::stages::GridGroundSegmentation;
using StatisticalGroundSegmentation = height_mapping::ppl::stages::StatisticalGroundSegmentation;
using MultiSensorSync = height_mapping::ppl::stages::MultiSensorSync;
using HeightEstimation = height_mapping::ppl::stages::HeightEstimation;
using MoveOrigin = height_mapping::ppl::stages::MoveOrigin;
using Raycasting = height_mapping::ppl::stages::Raycasting;

// Register all stages using PPL_REGISTER_STAGE macro
PPL_REGISTER_STAGE(MappingFrame, VoxelFilter, "VoxelFilter")
PPL_REGISTER_STAGE(MappingFrame, PassthroughFilter, "PassthroughFilter")
PPL_REGISTER_STAGE(MappingFrame, GridGroundSegmentation, "GridGroundSegmentation")
PPL_REGISTER_STAGE(MappingFrame, StatisticalGroundSegmentation, "StatisticalGroundSegmentation")
PPL_REGISTER_STAGE(MappingFrame, MultiSensorSync, "MultiSensorSync")
PPL_REGISTER_STAGE(MappingFrame, HeightEstimation, "HeightEstimation")
PPL_REGISTER_STAGE(MappingFrame, MoveOrigin, "MoveOrigin")
PPL_REGISTER_STAGE(MappingFrame, Raycasting, "Raycasting")

namespace height_mapping::ppl {

void registerAllStages() {
  // This function intentionally does nothing.
  // Its existence forces the linker to include this translation unit,
  // which triggers all PPL_REGISTER_STAGE macros at static init time.
}

}  // namespace height_mapping::ppl
