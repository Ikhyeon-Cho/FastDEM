/*
 * types.h
 *
 * Convenience typedefs for height mapping pipeline
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PPL_TYPES_H
#define HEIGHT_MAPPING_PPL_TYPES_H

#include <ppl/ppl.h>

#include "height_mapping/ppl/frame.h"

namespace height_mapping::ppl {

// Convenience typedefs for height mapping pipeline
using MappingStage = ::ppl::Stage<MappingFrame>;
using MappingPipeline = ::ppl::Pipeline<MappingFrame>;
using MappingProfiler = ::ppl::Profiler<MappingFrame>;

}  // namespace height_mapping::ppl

#endif  // HEIGHT_MAPPING_PPL_TYPES_H
