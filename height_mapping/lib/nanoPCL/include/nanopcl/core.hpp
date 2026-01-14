// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Core module: Point cloud container with metadata and attributes.
//
// Available types:
//   - PointCloud:  SoA container with lazy channel allocation
//   - Point:       3D point (Eigen::Vector3f)
//   - PointXYZ*:   PCL-style DTOs for convenient insertion
//   - Span<T>:     Non-owning view (C++17 std::span alternative)
//
// Attribute channels (enabled on demand):
//   - intensity, time, ring, color, normal, covariance, label

#ifndef NANOPCL_CORE_HPP
#define NANOPCL_CORE_HPP

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/core/span.hpp"

#endif  // NANOPCL_CORE_HPP
