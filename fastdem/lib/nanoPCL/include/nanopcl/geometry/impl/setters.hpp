// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Setter policies for normal/covariance estimation (Policy-based design).

#ifndef NANOPCL_GEOMETRY_IMPL_SETTERS_HPP
#define NANOPCL_GEOMETRY_IMPL_SETTERS_HPP

#include <Eigen/Core>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/core/types.hpp"

namespace nanopcl {
namespace geometry {
namespace detail {

/// @brief GICP covariance regularization constant
/// Forces planar assumption: eigenvalues = [epsilon, 1, 1]
constexpr float GICP_EPSILON = 1e-3f;

/// @brief Setter for normal estimation only
struct NormalSetter {
  Point viewpoint = Point::Zero();

  /// @brief Prepare point cloud channels
  static void prepare(PointCloud& cloud) {
    cloud.useNormal();
  }

  /// @brief Set invalid value (insufficient neighbors)
  static void setInvalid(PointCloud& cloud, size_t i) {
    cloud.normals()[i] = Normal4::Zero();
  }

  /// @brief Set normal from PCA result
  void set(PointCloud& cloud, size_t i, const Eigen::Matrix3f& eigenvectors, const Eigen::Vector3f& /*eigenvalues*/) const {
    // Normal = eigenvector corresponding to smallest eigenvalue (column 0)
    Eigen::Vector3f n = eigenvectors.col(0);

    // Orient towards viewpoint
    const Eigen::Vector3f to_viewpoint = viewpoint - cloud.point(i);
    if (n.dot(to_viewpoint) < 0) {
      n = -n;
    }

    cloud.normals()[i] = Normal4(n.x(), n.y(), n.z(), 0.0f);
  }
};

/// @brief Setter for covariance estimation only (GICP-ready)
struct CovarianceSetter {
  /// @brief Prepare point cloud channels
  static void prepare(PointCloud& cloud) {
    cloud.useCovariance();
  }

  /// @brief Set invalid value (insufficient neighbors)
  static void setInvalid(PointCloud& cloud, size_t i) {
    // Identity matrix for isotropic treatment in GICP
    cloud.covariances()[i] = Covariance::Identity();
  }

  /// @brief Set regularized covariance from PCA result
  void set(PointCloud& cloud, size_t i, const Eigen::Matrix3f& eigenvectors, const Eigen::Vector3f& /*eigenvalues*/) const {
    // GICP regularization: force planar assumption
    // Reconstruct covariance with [epsilon, 1, 1] eigenvalues
    const Eigen::Vector3f reg_values(GICP_EPSILON, 1.0f, 1.0f);
    cloud.covariances()[i] =
        eigenvectors * reg_values.asDiagonal() * eigenvectors.transpose();
  }
};

/// @brief Setter for both normal and covariance (optimized single pass)
struct NormalCovarianceSetter {
  Point viewpoint = Point::Zero();

  /// @brief Prepare point cloud channels
  static void prepare(PointCloud& cloud) {
    cloud.useNormal();
    cloud.useCovariance();
  }

  /// @brief Set invalid values (insufficient neighbors)
  static void setInvalid(PointCloud& cloud, size_t i) {
    cloud.normals()[i] = Normal4::Zero();
    cloud.covariances()[i] = Covariance::Identity();
  }

  /// @brief Set normal and covariance from PCA result
  void set(PointCloud& cloud, size_t i, const Eigen::Matrix3f& eigenvectors, const Eigen::Vector3f& eigenvalues) const {
    // Normal
    Eigen::Vector3f n = eigenvectors.col(0);
    const Eigen::Vector3f to_viewpoint = viewpoint - cloud.point(i);
    if (n.dot(to_viewpoint) < 0) {
      n = -n;
    }
    cloud.normals()[i] = Normal4(n.x(), n.y(), n.z(), 0.0f);

    // Covariance (GICP regularization)
    const Eigen::Vector3f reg_values(GICP_EPSILON, 1.0f, 1.0f);
    cloud.covariances()[i] =
        eigenvectors * reg_values.asDiagonal() * eigenvectors.transpose();
  }
};

} // namespace detail
} // namespace geometry
} // namespace nanopcl

#endif // NANOPCL_GEOMETRY_IMPL_SETTERS_HPP
