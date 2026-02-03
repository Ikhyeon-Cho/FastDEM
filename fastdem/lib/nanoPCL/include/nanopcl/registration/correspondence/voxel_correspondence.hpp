// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Voxel-based correspondence search for VGICP (O(1) lookup).

#ifndef NANOPCL_REGISTRATION_CORRESPONDENCE_VOXEL_CORRESPONDENCE_HPP
#define NANOPCL_REGISTRATION_CORRESPONDENCE_VOXEL_CORRESPONDENCE_HPP

#include <Eigen/Core>
#include <optional>

#include "nanopcl/registration/context.hpp"
#include "nanopcl/registration/voxel_distribution_map.hpp"

namespace nanopcl {
namespace registration {

/// @brief Voxel-based correspondence finder for VGICP
///
/// Provides O(1) hash-based lookup using VoxelDistributionMap.
/// Significantly faster than KdTree for large point clouds.
///
/// Usage:
/// @code
///   VoxelCorrespondence corr(voxel_resolution, epsilon);
///   corr.build(target);
///
///   auto ctx = corr.find(transformed_point, R, src_cov);
/// @endcode
class VoxelCorrespondence {
public:
  /// @brief Construct with voxel resolution
  /// @param resolution Voxel size in meters
  /// @param epsilon Covariance regularization epsilon (default: 1e-3)
  explicit VoxelCorrespondence(float resolution = 1.0f, double epsilon = 1e-3)
      : voxel_map_(resolution, epsilon) {}

  /// @brief Build voxel distribution map from target point cloud
  void build(const PointCloud& target) {
    voxel_map_.build(target);
  }

  /// @brief Check if map is empty
  [[nodiscard]] bool empty() const { return voxel_map_.empty(); }

  /// @brief Find correspondence for VGICP
  ///
  /// Uses O(1) hash lookup to find the voxel containing the transformed point.
  /// Returns the voxel's mean and pre-regularized covariance.
  ///
  /// @param p_transformed Transformed source point (T * p_source)
  /// @param R Current rotation matrix (for rotating source covariance)
  /// @param C_src_original Source covariance (will be rotated: R * C * R^T)
  /// @return DistributionContext if valid voxel found, nullopt otherwise
  [[nodiscard]] std::optional<DistributionContext> find(
      const Eigen::Vector3d& p_transformed,
      const Eigen::Matrix3d& R,
      const Eigen::Matrix3d& C_src_original) const {

    // O(1) voxel lookup - returns pre-regularized distribution
    auto dist = voxel_map_.lookupRegularized(p_transformed.cast<float>());
    if (!dist) {
      return std::nullopt;
    }

    // Rotate source covariance: C_src = R * C_original * R^T
    Eigen::Matrix3d C_src_rotated = R * C_src_original * R.transpose();

    return DistributionContext{
        p_transformed,
        dist->mean.cast<double>(),
        C_src_rotated,
        dist->covariance.cast<double>()
    };
  }

  /// @brief Access underlying voxel map
  [[nodiscard]] const VoxelDistributionMap& voxelMap() const { return voxel_map_; }

private:
  VoxelDistributionMap voxel_map_;
};

}  // namespace registration
}  // namespace nanopcl

#endif  // NANOPCL_REGISTRATION_CORRESPONDENCE_VOXEL_CORRESPONDENCE_HPP
