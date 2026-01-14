// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Local surface estimation implementation.
// Do not include this file directly; include <nanopcl/geometry/local_surface.hpp>

#ifndef NANOPCL_GEOMETRY_IMPL_LOCAL_SURFACE_IMPL_HPP
#define NANOPCL_GEOMETRY_IMPL_LOCAL_SURFACE_IMPL_HPP

#include <Eigen/Core>
#include <cmath>
#include <functional>
#include <limits>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/search/voxel_hash.hpp"

namespace npcl {
namespace geometry {
namespace detail {

// Constants
constexpr float TWO_PI_OVER_THREE = 2.0943951023931953f;

inline Eigen::Vector3f nanVector() {
  return Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
}

inline Eigen::Matrix3f nanMatrix() {
  return Eigen::Matrix3f::Constant(std::numeric_limits<float>::quiet_NaN());
}

/**
 * @brief Accumulator for neighbor statistics
 *
 * Collects running sums for centroid and covariance computation
 * in a single pass over neighbors.
 */
struct NeighborAccumulator {
  Point sum = Point::Zero();
  float cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
  size_t count = 0;

  void operator()(uint32_t, const Point& pt, float) {
    sum += pt;
    cxx += pt.x() * pt.x();
    cxy += pt.x() * pt.y();
    cxz += pt.x() * pt.z();
    cyy += pt.y() * pt.y();
    cyz += pt.y() * pt.z();
    czz += pt.z() * pt.z();
    count++;
  }

  /**
   * @brief Compute covariance matrix from accumulated statistics
   * @return 3x3 covariance matrix
   */
  Eigen::Matrix3f computeCovariance() const {
    float n = static_cast<float>(count);
    Point mean = sum / n;

    Eigen::Matrix3f cov;
    cov(0, 0) = cxx / n - mean.x() * mean.x();
    cov(0, 1) = cxy / n - mean.x() * mean.y();
    cov(0, 2) = cxz / n - mean.x() * mean.z();
    cov(1, 1) = cyy / n - mean.y() * mean.y();
    cov(1, 2) = cyz / n - mean.y() * mean.z();
    cov(2, 2) = czz / n - mean.z() * mean.z();
    cov(1, 0) = cov(0, 1);
    cov(2, 0) = cov(0, 2);
    cov(2, 1) = cov(1, 2);

    return cov;
  }
};

/**
 * @brief Extract normal from covariance matrix using analytical
 * eigendecomposition
 *
 * Uses closed-form solution for 3x3 symmetric eigenvalue problem.
 * Returns the eigenvector corresponding to the smallest eigenvalue.
 *
 * @param cov Covariance matrix
 * @param query_pt Query point (for normal orientation)
 * @param viewpoint Viewpoint for normal orientation
 * @return Surface normal vector (oriented towards viewpoint)
 */
inline Eigen::Vector3f extractNormalFromCovariance(const Eigen::Matrix3f& cov,
                                                   const Point& query_pt,
                                                   const Point& viewpoint) {
  // Scale for numerical stability
  float scale = cov.cwiseAbs().maxCoeff();
  if (scale <= std::numeric_limits<float>::min()) {
    return nanVector();
  }

  Eigen::Matrix3f scaled_cov = cov / scale;
  float m = (scaled_cov(0, 0) + scaled_cov(1, 1) + scaled_cov(2, 2)) / 3.0f;
  float q = (scaled_cov - m * Eigen::Matrix3f::Identity()).squaredNorm() / 6.0f;
  float det_b = (scaled_cov - m * Eigen::Matrix3f::Identity()).determinant();

  Point normal;
  if (q <= std::numeric_limits<float>::min()) {
    normal = Point::UnitX();
  } else {
    float phi =
        std::atan2(std::sqrt(std::max(0.0f, 4.0f * q * q * q - det_b * det_b)),
                   det_b) /
        3.0f;
    float eigenvalue =
        m + 2.0f * std::sqrt(q) * std::cos(phi + TWO_PI_OVER_THREE);

    Eigen::Matrix3f r = scaled_cov - eigenvalue * Eigen::Matrix3f::Identity();
    Point r0 = r.row(0), r1 = r.row(1), r2 = r.row(2);
    Point n0 = r0.cross(r1), n1 = r0.cross(r2), n2 = r1.cross(r2);
    float d0 = n0.squaredNorm(), d1 = n1.squaredNorm(), d2 = n2.squaredNorm();

    float max_d = d0;
    normal = n0;
    if (d1 > max_d) {
      max_d = d1;
      normal = n1;
    }
    if (d2 > max_d) {
      max_d = d2;
      normal = n2;
    }

    if (max_d <= std::numeric_limits<float>::min()) {
      normal = Point::UnitX();
    } else {
      normal.normalize();
    }
  }

  // Orient towards viewpoint
  Eigen::Vector3f to_viewpoint = viewpoint - query_pt;
  if (normal.dot(to_viewpoint) < 0) {
    normal = -normal;
  }

  return normal;
}

/**
 * @brief Unified kernel for local surface computation
 *
 * Template parameters control which outputs are computed:
 * - ComputeNormal: Extract surface normal from covariance
 * - ComputeCovariance: Store raw covariance matrix
 *
 * This design allows sharing the expensive neighbor search and
 * covariance computation between normal and covariance estimation.
 *
 * @tparam ComputeNormal Whether to compute and store normals
 * @tparam ComputeCovariance Whether to compute and store covariances
 * @param cloud Point cloud to process (modified in-place)
 * @param radius Search radius for neighbor finding
 * @param min_neighbors Minimum neighbors for valid computation
 * @param viewpoint Viewpoint for normal orientation (only used if ComputeNormal)
 */
template <bool ComputeNormal, bool ComputeCovariance>
void computeLocalSurface(PointCloud& cloud, float radius, size_t min_neighbors,
                         const Point& viewpoint = Point::Zero()) {
  static_assert(ComputeNormal || ComputeCovariance,
                "At least one output must be enabled");

  if (cloud.empty()) return;

  // Enable required channels and initialize with invalid values
  if constexpr (ComputeNormal) {
    cloud.enableNormal();
    std::fill(cloud.normal().begin(), cloud.normal().end(), nanVector());
  }
  if constexpr (ComputeCovariance) {
    cloud.enableCovariance();
    std::fill(cloud.covariance().begin(), cloud.covariance().end(),
              nanMatrix());
  }

  // Build spatial search structure
  search::VoxelHash searcher(radius);
  searcher.build(cloud);

#pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& query_pt = cloud[i];

    if (!query_pt.allFinite()) continue;

    // Accumulate neighbor statistics
    NeighborAccumulator acc;
    searcher.radius(query_pt, radius, std::ref(acc));

    if (acc.count < min_neighbors) continue;

    // Compute covariance matrix (always needed for normal extraction)
    Eigen::Matrix3f cov = acc.computeCovariance();

    // Store covariance if requested
    if constexpr (ComputeCovariance) {
      cloud.covariance()[i] = cov;
    }

    // Extract and store normal if requested
    if constexpr (ComputeNormal) {
      cloud.normal()[i] = extractNormalFromCovariance(cov, query_pt, viewpoint);
    }
  }
}

}  // namespace detail

// =============================================================================
// Public API implementations
// =============================================================================

inline void estimateNormals(PointCloud& cloud, float radius,
                            size_t min_neighbors, const Point& viewpoint) {
  detail::computeLocalSurface<true, false>(cloud, radius, min_neighbors,
                                           viewpoint);
}

inline void estimateCovariances(PointCloud& cloud, float radius,
                                size_t min_neighbors) {
  detail::computeLocalSurface<false, true>(cloud, radius, min_neighbors);
}

inline void estimateLocalSurface(PointCloud& cloud, float radius,
                                 size_t min_neighbors, const Point& viewpoint) {
  detail::computeLocalSurface<true, true>(cloud, radius, min_neighbors,
                                          viewpoint);
}

}  // namespace geometry
}  // namespace npcl

#endif  // NANOPCL_GEOMETRY_IMPL_LOCAL_SURFACE_IMPL_HPP
