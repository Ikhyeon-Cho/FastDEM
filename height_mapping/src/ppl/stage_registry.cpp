// Stage registration for height mapping pipeline.
// The empty registerAllStages() forces linker to include this unit.

#include "height_mapping/ppl/stage_registry.h"

#include <ppl/ppl.h>

#include "height_mapping/ppl/frame.h"
#include "height_mapping/ppl/stages/height_estimation.h"
#include "height_mapping/ppl/stages/inpainting.h"
#include "height_mapping/ppl/stages/move_origin.h"
#include "height_mapping/ppl/stages/multi_sensor_sync.h"
#include "height_mapping/ppl/stages/passthrough_filter.h"
#include "height_mapping/ppl/stages/raycasting.h"
#include "height_mapping/ppl/stages/voxel_filter.h"

using namespace height_mapping::ppl;
using namespace height_mapping::ppl::stages;

// clang-format off
PPL_REGISTER_STAGE(MappingFrame, VoxelFilter, "VoxelFilter")
PPL_REGISTER_STAGE(MappingFrame, PassthroughFilter, "PassthroughFilter")
PPL_REGISTER_STAGE(MappingFrame, MultiSensorSync, "MultiSensorSync")
PPL_REGISTER_STAGE(MappingFrame, HeightEstimation, "HeightEstimation")
PPL_REGISTER_STAGE(MappingFrame, MoveOrigin, "MoveOrigin")
PPL_REGISTER_STAGE(MappingFrame, Raycasting, "Raycasting")
PPL_REGISTER_STAGE(MappingFrame, Inpainting, "Inpainting")
// clang-format on

namespace height_mapping::ppl {
void registerAllStages() {}
}  // namespace height_mapping::ppl
