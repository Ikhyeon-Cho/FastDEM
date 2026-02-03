// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Filters module: Point cloud filtering operations.
//
// Available filters:
//   - voxelGrid():      Downsample using voxel grid (multiple methods)
//   - cropBox():        Axis-aligned bounding box filter
//   - cropRange():      Distance-based ring filter
//   - cropX/Y/Z():      Single-axis passthrough filter
//   - cropAngle():      Azimuth angle (horizontal FOV) filter
//   - filter():         Custom predicate filter
//   - removeInvalid():  Remove NaN/Inf points
//   - removeOutliers(): Statistical outlier removal
//   - deskew():         Motion distortion correction

#ifndef NANOPCL_FILTERS_HPP
#define NANOPCL_FILTERS_HPP

#include "nanopcl/filters/core.hpp"
#include "nanopcl/filters/crop.hpp"
#include "nanopcl/filters/deskew.hpp"
#include "nanopcl/filters/downsample.hpp"
#include "nanopcl/filters/outlier_removal.hpp"

#endif // NANOPCL_FILTERS_HPP
