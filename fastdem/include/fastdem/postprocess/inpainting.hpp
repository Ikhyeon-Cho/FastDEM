// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * inpainting.hpp
 *
 * Iterative neighbor averaging for filling NaN holes in height maps.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *       Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_POSTPROCESS_INPAINTING_HPP
#define FASTDEM_POSTPROCESS_INPAINTING_HPP

#include "fastdem/config/inpainting.hpp"
#include "fastdem/elevation_map.hpp"

namespace fastdem {

/**
 * @brief Fills NaN holes in elevation using neighbor averaging.
 *
 * Results stored in elevation_inpainted layer (original unchanged).
 *
 * @param map Height map to process
 * @param config Inpainting configuration
 */
void applyInpainting(ElevationMap& map, const config::Inpainting& config);

}  // namespace fastdem

#endif  // FASTDEM_POSTPROCESS_INPAINTING_HPP
