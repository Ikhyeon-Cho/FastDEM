#pragma once

namespace height_mapping::ppl {

/**
 * Forces linker to include stage_registry.o (which contains PPL_REGISTER_STAGE
 * macros). This empty function exists because:
 *
 * - PPL_REGISTER_STAGE uses static variable initialization for auto-registration
 * - __attribute__((used)) only prevents compiler optimization, not linker stripping
 * - Static library linking discards unreferenced .o files entirely
 *
 * Must be called once before Pipeline::load(). Typically called in HeightMapper
 * constructor.
 */
void registerAllStages();

}  // namespace height_mapping::ppl
