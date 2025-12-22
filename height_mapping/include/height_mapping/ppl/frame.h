/*
 * frame.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_PPL_FRAME_H
#define HEIGHT_MAPPING_PPL_FRAME_H

#include <memory>

#include "height_mapping/height_map.h"
#include "height_mapping/types.h"

namespace height_mapping::ppl {

/// Pipeline processing frame - contains data, pose, and map for a single scan

struct MappingFrame {
  using Ptr = std::shared_ptr<MappingFrame>;
  std::shared_ptr<PointCloud> cloud;
  std::shared_ptr<HeightMap> map;

  Transformf pose;       // Robot pose in map frame (T_map_base)
  Transformf extrinsic;  // Sensor extrinsic (T_base_sensor)

  MappingFrame(std::shared_ptr<PointCloud> c, std::shared_ptr<HeightMap> m)
      : cloud(std::move(c)), map(std::move(m)) {}
};

}  // namespace height_mapping::ppl

#endif  // HEIGHT_MAPPING_PPL_FRAME_H
