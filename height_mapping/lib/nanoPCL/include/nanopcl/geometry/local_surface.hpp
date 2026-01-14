// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT

#ifndef NANOPCL_GEOMETRY_LOCAL_SURFACE_HPP
#define NANOPCL_GEOMETRY_LOCAL_SURFACE_HPP

#include "nanopcl/core/point_cloud.hpp"

namespace npcl {
namespace geometry {

/**
 * @brief Estimate surface normals using PCA (in-place)
 *
 * Computes normal vectors by performing Principal Component Analysis (PCA)
 * on the neighborhood of each point.
 *
 * - **Algorithm**: The normal is the eigenvector corresponding to the smallest
 *   eigenvalue of the local covariance matrix.
 * - **Orientation**: Normals are flipped to point towards the `viewpoint`.
 * - **Parallelization**: Uses **OpenMP** for multi-threaded computation.
 *
 * Results are stored in cloud.normal() channel.
 * Invalid normals are set to NaN (check with allFinite()).
 *
 * @param cloud Point cloud (normal channel will be enabled/overwritten)
 * @param radius Search radius for neighbor finding
 * @param min_neighbors Minimum neighbors for valid normal (default: 5)
 * @param viewpoint Viewpoint for normal orientation (default: origin)
 */
void estimateNormals(PointCloud& cloud, float radius, size_t min_neighbors = 5,
                     const Point& viewpoint = Point::Zero());

/**
 * @brief Estimate local covariance matrices (in-place)
 *
 * Computes the 3x3 covariance matrix for each point based on its neighbors.
 * Results are stored in cloud.covariance() channel.
 * Invalid covariances (insufficient neighbors) are set to NaN.
 *
 * @param cloud Point cloud (covariance channel will be enabled/overwritten)
 * @param radius Search radius for neighbor finding
 * @param min_neighbors Minimum neighbors for valid covariance (default: 5)
 */
void estimateCovariances(PointCloud& cloud, float radius,
                         size_t min_neighbors = 5);

/**
 * @brief Estimate both normals and covariances (in-place, optimized)
 *
 * More efficient than calling estimateNormals() and estimateCovariances()
 * separately, as it performs the neighbor search only once.
 *
 * Results stored in cloud.normal() and cloud.covariance().
 * Invalid values are set to NaN.
 * Normals are oriented towards the viewpoint.
 *
 * @param cloud Point cloud (normal and covariance channels will be enabled)
 * @param radius Search radius for neighbor finding
 * @param min_neighbors Minimum neighbors for valid computation (default: 5)
 * @param viewpoint Viewpoint for normal orientation (default: origin)
 */
void estimateLocalSurface(PointCloud& cloud, float radius,
                          size_t min_neighbors = 5,
                          const Point& viewpoint = Point::Zero());

}  // namespace geometry
}  // namespace npcl

#include "nanopcl/geometry/impl/local_surface_impl.hpp"

#endif  // NANOPCL_GEOMETRY_LOCAL_SURFACE_HPP
