// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Registration module: Point cloud alignment algorithms.
//
// Available algorithms:
//   - alignICP:       Point-to-Point ICP (general purpose)
//   - alignPlaneICP:  Point-to-Plane ICP (faster on planar surfaces)
//   - alignGICP:      Generalized ICP (plane-to-plane, requires covariances)
//   - alignVGICP:     Voxelized GICP (fastest, requires source covariances)
//
// Example usage:
//   #include <nanopcl/registration.hpp>
//
//   // Point-to-Point ICP
//   auto result = registration::alignICP(source, target);
//
//   // Point-to-Plane ICP (requires normals on target)
//   geometry::estimateNormals(target, 20);
//   auto result = registration::alignPlaneICP(source, target);
//
//   // GICP (requires covariances on both clouds)
//   geometry::estimateCovariances(source, 20);
//   geometry::estimateCovariances(target, 20);
//   auto result = registration::alignGICP(source, target);
//
//   // VGICP with prebuilt map (fastest for SLAM)
//   geometry::estimateCovariances(source, 20);
//   registration::VoxelCorrespondence voxel_map(1.0f);
//   voxel_map.build(target);
//   auto result = registration::alignVGICP(source, voxel_map);

#ifndef NANOPCL_REGISTRATION_HPP
#define NANOPCL_REGISTRATION_HPP

// Main API - convenience functions
#include "nanopcl/registration/align.hpp"

// Result and settings
#include "nanopcl/registration/result.hpp"

// Advanced: direct solver access
#include "nanopcl/registration/iterative_solver.hpp"

// Advanced: correspondence finders
#include "nanopcl/registration/correspondence/kdtree_correspondence.hpp"
#include "nanopcl/registration/correspondence/voxel_correspondence.hpp"

// Advanced: factors (for custom registration pipelines)
#include "nanopcl/registration/factors/icp_factor.hpp"
#include "nanopcl/registration/factors/plane_factor.hpp"
#include "nanopcl/registration/factors/gicp_factor.hpp"
#include "nanopcl/registration/factors/vgicp_factor.hpp"

// Advanced: VGICP voxel map
#include "nanopcl/registration/voxel_distribution_map.hpp"

#endif // NANOPCL_REGISTRATION_HPP
