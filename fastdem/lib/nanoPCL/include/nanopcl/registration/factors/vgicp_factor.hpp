// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// VGICP (Voxelized Generalized ICP) factor.
// Uses the same error function as GICP, but with O(1) voxel-based lookup.

#ifndef NANOPCL_REGISTRATION_FACTORS_VGICP_FACTOR_HPP
#define NANOPCL_REGISTRATION_FACTORS_VGICP_FACTOR_HPP

#include "nanopcl/registration/factors/gicp_factor.hpp"

namespace nanopcl {
namespace registration {

/// @brief VGICP factor for voxelized distribution-to-distribution alignment
///
/// Mathematically identical to GICPFactor. The difference is in how
/// correspondences are found (VoxelCorrespondence uses O(1) hash lookup
/// instead of KdTree's O(log N) search).
///
/// Both factors use the same DistributionContext and linearize() function.
using VGICPFactor = GICPFactor;

}  // namespace registration
}  // namespace nanopcl

#endif  // NANOPCL_REGISTRATION_FACTORS_VGICP_FACTOR_HPP
