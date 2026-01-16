/*
 * frame.h
 *
 * Pipeline context and convenience typedefs for lab module.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_LAB_FRAME_H
#define HEIGHT_MAPPING_LAB_FRAME_H

#include <ppl/ppl.h>

#include <memory>

#include "height_mapping/core.h"

namespace height_mapping::lab {

/// Pipeline processing frame - contains data, pose, and map for a single scan
struct MappingFrame {
  using Ptr = std::shared_ptr<MappingFrame>;

  std::shared_ptr<PointCloud> cloud;
  std::shared_ptr<HeightMap> height_map;

  Transformf robot_pose;  // Robot pose in map frame (T_map_base)
  Transformf extrinsic;   // Sensor extrinsic (T_base_sensor)

  MappingFrame(std::shared_ptr<PointCloud> c, std::shared_ptr<HeightMap> m)
      : cloud(std::move(c)), height_map(std::move(m)) {}
};

// Convenience typedefs for height mapping pipeline
using MappingStage = ::ppl::Stage<MappingFrame>;
using MappingPipeline = ::ppl::Pipeline<MappingFrame>;
using MappingProfiler = ::ppl::Profiler<MappingFrame>;

}  // namespace height_mapping::lab

#endif  // HEIGHT_MAPPING_LAB_FRAME_H
