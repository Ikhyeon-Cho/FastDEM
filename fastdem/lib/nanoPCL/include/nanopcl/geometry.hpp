// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Geometry module: Surface geometry estimation algorithms.
//
// Available functions:
//   - estimateNormals():      Surface normal estimation (PCA-based)
//   - estimateCovariances():  Local covariance estimation (for GICP)
//
// OpenMP variants (parallel):
//   - estimateNormalsOMP():
//   - estimateCovariancesOMP():

#ifndef NANOPCL_GEOMETRY_HPP
#define NANOPCL_GEOMETRY_HPP

#include "nanopcl/geometry/normal_estimation.hpp"

#ifdef _OPENMP
#include "nanopcl/geometry/normal_estimation_omp.hpp"
#endif

#endif // NANOPCL_GEOMETRY_HPP
