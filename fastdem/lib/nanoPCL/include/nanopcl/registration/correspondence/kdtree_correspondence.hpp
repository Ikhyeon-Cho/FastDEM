// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// KdTree-based correspondence search for ICP, PlaneICP, and GICP.

#ifndef NANOPCL_REGISTRATION_CORRESPONDENCE_KDTREE_CORRESPONDENCE_HPP
#define NANOPCL_REGISTRATION_CORRESPONDENCE_KDTREE_CORRESPONDENCE_HPP

#include <Eigen/Core>
#include <cmath>
#include <optional>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/registration/context.hpp"
#include "nanopcl/search/kdtree.hpp"

namespace nanopcl {
namespace registration {

/// @brief KdTree-based correspondence finder
///
/// Provides O(log N) nearest neighbor search for various registration algorithms.
/// Returns appropriate Context type based on what data is needed.
///
/// Usage:
/// @code
///   KdTreeCorrespondence corr;
///   corr.build(target);
///
///   // For ICP
///   auto ctx = corr.find(transformed_point, max_dist_sq);
///
///   // For PlaneICP (requires target normals)
///   auto ctx = corr.findWithNormal(transformed_point, max_dist_sq);
///
///   // For GICP (requires covariances)
///   auto ctx = corr.findWithCovariance(transformed_point, R, src_cov, max_dist_sq);
/// @endcode
class KdTreeCorrespondence {
public:
  /// @brief Build search index from target point cloud
  void build(const PointCloud& target) {
    target_ = &target;
    tree_.build(target);
  }

  /// @brief Set pre-computed regularized covariances for GICP
  void setRegularizedCovariances(const std::vector<Eigen::Matrix3d>* target_cov) {
    target_cov_reg_ = target_cov;
  }

  /// @brief Find correspondence for Point-to-Point ICP
  /// @param p_transformed Transformed source point (T * p_source)
  /// @param max_dist_sq Maximum squared distance for valid correspondence
  /// @return PointContext if valid correspondence found, nullopt otherwise
  [[nodiscard]] std::optional<PointContext> find(
      const Eigen::Vector3d& p_transformed,
      float max_dist_sq) const {
    auto nearest = tree_.nearest(p_transformed.cast<float>(), std::sqrt(max_dist_sq));
    if (!nearest || nearest->dist_sq > max_dist_sq) {
      return std::nullopt;
    }

    return PointContext{
        p_transformed,
        (*target_)[nearest->index].head<3>().cast<double>()
    };
  }

  /// @brief Find correspondence for Point-to-Plane ICP
  /// @param p_transformed Transformed source point (T * p_source)
  /// @param max_dist_sq Maximum squared distance for valid correspondence
  /// @return PlaneContext if valid correspondence found, nullopt otherwise
  /// @pre Target must have normals
  [[nodiscard]] std::optional<PlaneContext> findWithNormal(
      const Eigen::Vector3d& p_transformed,
      float max_dist_sq) const {
    auto nearest = tree_.nearest(p_transformed.cast<float>(), std::sqrt(max_dist_sq));
    if (!nearest || nearest->dist_sq > max_dist_sq) {
      return std::nullopt;
    }

    const size_t idx = nearest->index;
    return PlaneContext{
        p_transformed,
        (*target_)[idx].head<3>().cast<double>(),
        target_->normal(idx).cast<double>()
    };
  }

  /// @brief Find correspondence for GICP
  /// @param p_transformed Transformed source point (T * p_source)
  /// @param R Current rotation matrix (for rotating source covariance)
  /// @param C_src_original Source covariance (will be rotated: R * C * R^T)
  /// @param max_dist_sq Maximum squared distance for valid correspondence
  /// @return DistributionContext if valid correspondence found, nullopt otherwise
  /// @pre setRegularizedCovariances() must be called first
  [[nodiscard]] std::optional<DistributionContext> findWithCovariance(
      const Eigen::Vector3d& p_transformed,
      const Eigen::Matrix3d& R,
      const Eigen::Matrix3d& C_src_original,
      float max_dist_sq) const {
    auto nearest = tree_.nearest(p_transformed.cast<float>(), std::sqrt(max_dist_sq));
    if (!nearest || nearest->dist_sq > max_dist_sq) {
      return std::nullopt;
    }

    const size_t idx = nearest->index;

    // Rotate source covariance: C_src = R * C_original * R^T
    Eigen::Matrix3d C_src_rotated = R * C_src_original * R.transpose();

    return DistributionContext{
        p_transformed,
        (*target_)[idx].head<3>().cast<double>(),
        C_src_rotated,
        (*target_cov_reg_)[idx]
    };
  }

  /// @brief Get target index from last search (for debugging/tracking)
  [[nodiscard]] const search::KdTree& tree() const { return tree_; }

private:
  const PointCloud* target_ = nullptr;
  const std::vector<Eigen::Matrix3d>* target_cov_reg_ = nullptr;
  search::KdTree tree_;
};

}  // namespace registration
}  // namespace nanopcl

#endif  // NANOPCL_REGISTRATION_CORRESPONDENCE_KDTREE_CORRESPONDENCE_HPP
