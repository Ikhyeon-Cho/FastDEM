// Stage registration for height mapping pipeline.
// The empty registerAllStages() forces linker to include this unit.

#include "height_mapping/lab/stage_registry.h"

#include <ppl/ppl.h>

#include "height_mapping/lab/frame.h"
#include "height_mapping/lab/stages/stages.h"

using namespace height_mapping::lab;
using namespace height_mapping::lab::stages;

// clang-format off
PPL_REGISTER_STAGE(MappingFrame, VoxelFilter, "VoxelFilter")
PPL_REGISTER_STAGE(MappingFrame, PassthroughFilter, "PassthroughFilter")
PPL_REGISTER_STAGE(MappingFrame, MultiSensorSync, "MultiSensorSync")
PPL_REGISTER_STAGE(MappingFrame, HeightEstimation, "HeightEstimation")
PPL_REGISTER_STAGE(MappingFrame, MoveOrigin, "MoveOrigin")
PPL_REGISTER_STAGE(MappingFrame, Raycasting, "Raycasting")
PPL_REGISTER_STAGE(MappingFrame, Inpainting, "Inpainting")
// clang-format on

namespace height_mapping::lab {
void registerAllStages() {}
}  // namespace height_mapping::lab
