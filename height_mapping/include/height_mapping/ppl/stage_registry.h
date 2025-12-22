/*
 * stage_registry.h
 *
 * Explicit stage registration for height mapping pipeline.
 * This avoids Static Initialization Order Fiasco with ppl macros.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PPL_STAGE_REGISTRY_H
#define HEIGHT_MAPPING_PPL_STAGE_REGISTRY_H

namespace height_mapping::ppl {

/**
 * @brief Register all pipeline stages with ppl::Registry
 *
 * This function must be called before loading pipeline from YAML.
 * It ensures all stages are registered regardless of linker optimization.
 */
void registerAllStages();

}  // namespace height_mapping::ppl

#endif  // HEIGHT_MAPPING_PPL_STAGE_REGISTRY_H
