// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Registration module: Point cloud alignment algorithms.
//
// Available algorithms:
//   - icp():      Point-to-Point ICP (general purpose)
//   - icpPlane(): Point-to-Plane ICP (faster on planar surfaces)
//   - gicp():     Generalized ICP (plane-to-plane, requires covariances)
//   - vgicp():    Voxelized GICP (fastest, requires source covariances)
//
// Example usage:
//   #include <nanopcl/registration.hpp>
//
//   // Point-to-Point ICP
//   auto result = registration::icp(source, target);
//
//   // Point-to-Plane ICP (requires normals on target)
//   geometry::estimateNormals(target, 0.3f);
//   auto result = registration::icpPlane(source, target);
//
//   // GICP (requires covariances on both clouds)
//   geometry::estimateCovariances(source, 0.3f);
//   geometry::estimateCovariances(target, 0.3f);
//   auto result = registration::gicp(source, target);
//
//   // VGICP (fastest - voxelized target, source covariances only)
//   geometry::estimateCovariances(source, 0.3f);
//   registration::VoxelDistributionMap voxel_map(0.5f);
//   voxel_map.build(target);
//   auto result = registration::vgicp(source, voxel_map);

#ifndef NANOPCL_REGISTRATION_HPP
#define NANOPCL_REGISTRATION_HPP

#include "nanopcl/registration/correspondence.hpp"
#include "nanopcl/registration/gicp.hpp"
#include "nanopcl/registration/icp.hpp"
#include "nanopcl/registration/icp_plane.hpp"
#include "nanopcl/registration/result.hpp"
#include "nanopcl/registration/vgicp.hpp"

#endif  // NANOPCL_REGISTRATION_HPP
