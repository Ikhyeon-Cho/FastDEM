// nanoPCL - Core module aggregate header
// SPDX-License-Identifier: MIT
//
// Core module: Point cloud container, transformation, and utilities.
//
// Available types:
//   - PointCloud:  SoA container with lazy channel allocation
//   - Point:       3D point (Eigen::Vector3f)
//   - Span<T>:     Non-owning view (C++17 std::span alternative)
//
// Utilities:
//   - transformCloud(): Rigid body transformation
//   - math::*:          RPY, slerp, angle conversions
//   - voxel::*:         Voxel key packing/unpacking

#ifndef NANOPCL_CORE_HPP
#define NANOPCL_CORE_HPP

#include "nanopcl/core/math.hpp"
#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/core/span.hpp"
#include "nanopcl/core/transform.hpp"
#include "nanopcl/core/voxel.hpp"

#endif // NANOPCL_CORE_HPP
