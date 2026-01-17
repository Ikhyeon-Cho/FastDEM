/*
 * inpainting.h
 *
 * Iterative neighbor averaging for filling NaN holes in height maps.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_OPS_INPAINTING_H
#define HEIGHT_MAPPING_OPS_INPAINTING_H

#include "height_mapping/config/types.h"
#include "height_mapping/core/map.h"

namespace height_mapping::ops {

/**
 * @brief Fills NaN holes in elevation using neighbor averaging.
 *
 * Results stored in elevation_inpainted layer (original unchanged).
 *
 * @param map Height map to process
 * @param config Inpainting configuration
 */
void applyInpainting(HeightMap& map, const config::Inpainting& config);

}  // namespace height_mapping::ops

#endif  // HEIGHT_MAPPING_OPS_INPAINTING_H
