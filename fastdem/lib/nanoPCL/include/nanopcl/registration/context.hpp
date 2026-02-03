// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Evaluation contexts for registration factors.
// Context pattern separates correspondence lookup from error computation.

#ifndef NANOPCL_REGISTRATION_CONTEXT_HPP
#define NANOPCL_REGISTRATION_CONTEXT_HPP

#include <Eigen/Core>

namespace nanopcl {
namespace registration {

// =======================================
// Point Context (for Point-to-Point ICP)
// =======================================

/// @brief Context for point-to-point alignment
///
/// Contains transformed source point and corresponding target point.
/// Used by ICPFactor for simple point-to-point error computation.
struct PointContext {
  Eigen::Vector3d p_src;  ///< Transformed source point (T * p_source)
  Eigen::Vector3d p_tgt;  ///< Corresponding target point
};

// =======================================
// Plane Context (for Point-to-Plane ICP)
// =======================================

/// @brief Context for point-to-plane alignment
///
/// Contains transformed source point, target point, and target surface normal.
/// Used by PlaneICPFactor for point-to-plane error computation.
struct PlaneContext {
  Eigen::Vector3d p_src;  ///< Transformed source point (T * p_source)
  Eigen::Vector3d p_tgt;  ///< Corresponding target point
  Eigen::Vector3d n_tgt;  ///< Target surface normal (unit vector)
};

// ========================================
// Distribution Context (for GICP / VGICP)
// ========================================

/// @brief Context for distribution-to-distribution alignment
///
/// Contains transformed source point, target mean, and covariances.
/// Used by GICPFactor and VGICPFactor for Mahalanobis distance computation.
///
/// @note C_src must be pre-rotated: C_src = R * C_source_original * R^T
///       This rotation is performed by the Correspondence module, not Factor.
struct DistributionContext {
  Eigen::Vector3d p_src;   ///< Transformed source point (T * p_source)
  Eigen::Vector3d p_tgt;   ///< Target mean (or corresponding point)
  Eigen::Matrix3d C_src;   ///< Source covariance (already rotated: R * C * R^T)
  Eigen::Matrix3d C_tgt;   ///< Target covariance (regularized)
};

}  // namespace registration
}  // namespace nanopcl

#endif  // NANOPCL_REGISTRATION_CONTEXT_HPP
