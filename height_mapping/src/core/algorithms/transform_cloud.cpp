/*
 * transform_cloud.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/core/algorithms/transform_cloud.h"

namespace height_mapping::algorithms {

TransformCloud::Stats TransformCloud::transformInPlace(
    height_mapping::PointCloud &cloud,
    const height_mapping::Transform3D &transform) const {
  Stats stats;
  stats.original_frame = cloud.frameId();
  stats.target_frame = config_.target_frame;

  // Skip if already in target frame
  if (config_.skip_if_same_frame && cloud.frameId() == config_.target_frame) {
    stats.transform_applied = false;
    return stats;
  }

  // Apply transform to all points
  const auto &T = transform.matrix();
  for (auto point : cloud) {
    float x = point.x();
    float y = point.y();
    float z = point.z();

    point.x() = T(0, 0) * x + T(0, 1) * y + T(0, 2) * z + T(0, 3);
    point.y() = T(1, 0) * x + T(1, 1) * y + T(1, 2) * z + T(1, 3);
    point.z() = T(2, 0) * x + T(2, 1) * y + T(2, 2) * z + T(2, 3);
  }

  // Update frame_id after successful transform
  cloud.setFrameId(config_.target_frame);

  stats.transform_applied = true;
  stats.points_transformed = cloud.size();

  return stats;
}

height_mapping::PointCloud TransformCloud::transform(
    const height_mapping::PointCloud &cloud,
    const height_mapping::Transform3D &transform, Stats &stats) const {
  // Create a copy and transform it
  height_mapping::PointCloud result = cloud;
  stats = transformInPlace(result, transform);
  return result;
}

bool TransformCloud::needsTransform(
    const height_mapping::PointCloud &cloud) const {
  return !cloud.empty() && cloud.frameId() != config_.target_frame;
}

}  // namespace height_mapping::algorithms