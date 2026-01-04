// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// This file contains implementation details for normal estimation.
// Do not include this file directly; include <nanopcl/geometry/normal_estimation.hpp>

#ifndef NANOPCL_GEOMETRY_IMPL_NORMAL_ESTIMATION_IMPL_HPP
#define NANOPCL_GEOMETRY_IMPL_NORMAL_ESTIMATION_IMPL_HPP

#include <Eigen/Core>
#include <cmath>
#include <functional>
#include <limits>

namespace nanopcl {
namespace geometry {

namespace detail {
/// 2*pi/3 radians (120 degrees) - used in eigenvalue computation
constexpr float TWO_PI_OVER_THREE = 2.0943951023931953f;
}  // namespace detail

template <typename SearcherT>
inline void estimateNormals(const PointCloud& cloud, const SearcherT& searcher,
                            std::vector<Point>& normals,
                            const NormalEstimationConfig& config) {
  normals.resize(cloud.size());
  std::vector<uint32_t> neighbors;
  neighbors.reserve(100);  // Pre-allocate for typical density

  for (size_t i = 0; i < cloud.size(); ++i) {
    const auto& query_pt = cloud[i];

    // Default to zero normal if invalid
    normals[i] = Point::Zero();

    if (!query_pt.allFinite()) continue;

    // 1. Find neighbors and compute stats in a single pass using callback
    struct Accumulator {
      Point sum = Point::Zero();
      float cxx = 0, cxy = 0, cxz = 0, cyy = 0, cyz = 0, czz = 0;
      size_t count = 0;

      void operator()(uint32_t /*idx*/, const Point& pt) {
        sum += pt;
        cxx += pt.x() * pt.x();
        cxy += pt.x() * pt.y();
        cxz += pt.x() * pt.z();
        cyy += pt.y() * pt.y();
        cyz += pt.y() * pt.z();
        czz += pt.z() * pt.z();
        count++;
      }
    } acc;

    searcher.radiusSearch(query_pt, config.radius, std::ref(acc));

    if (acc.count < config.min_neighbors) {
      continue;
    }

    // 2. Compute Covariance Matrix
    float n = static_cast<float>(acc.count);
    Point mean = acc.sum / n;

    Eigen::Matrix3f cov;
    cov(0, 0) = acc.cxx / n - mean.x() * mean.x();
    cov(0, 1) = acc.cxy / n - mean.x() * mean.y();
    cov(0, 2) = acc.cxz / n - mean.x() * mean.z();
    cov(1, 1) = acc.cyy / n - mean.y() * mean.y();
    cov(1, 2) = acc.cyz / n - mean.y() * mean.z();
    cov(2, 2) = acc.czz / n - mean.z() * mean.z();
    cov(1, 0) = cov(0, 1);
    cov(2, 0) = cov(0, 2);
    cov(2, 1) = cov(1, 2);

    // 3. Analytical 3x3 Eigen Decomposition (Fastest for 3x3)
    Point normal;

    // Scale matrix to improve numerical stability
    float scale = cov.cwiseAbs().maxCoeff();
    if (scale <= std::numeric_limits<float>::min()) {
      normals[i] = Point::Zero();
      continue;
    }
    Eigen::Matrix3f scaled_cov = cov / scale;

    // Characteristic equation: det(A - lambda*I) = 0
    float m = (scaled_cov(0, 0) + scaled_cov(1, 1) + scaled_cov(2, 2)) / 3.0f;
    float q = (scaled_cov - m * Eigen::Matrix3f::Identity()).squaredNorm() / 6.0f;
    float det_b = (scaled_cov - m * Eigen::Matrix3f::Identity()).determinant();

    float eigenvalue;
    if (q <= std::numeric_limits<float>::min()) {
      eigenvalue = m;
      normal = Point::UnitX();  // Arbitrary
    } else {
      float phi = std::atan2(
          std::sqrt(std::max(0.0f, 4.0f * q * q * q - det_b * det_b)), det_b) / 3.0f;
      // Smallest eigenvalue is m + 2*sqrt(q)*cos(phi + 2*pi/3)
      eigenvalue = m + 2.0f * std::sqrt(q) * std::cos(phi + detail::TWO_PI_OVER_THREE);

      // Compute eigenvector for this eigenvalue using cross product
      Eigen::Matrix3f r = scaled_cov - eigenvalue * Eigen::Matrix3f::Identity();
      Point r0 = r.row(0);
      Point r1 = r.row(1);
      Point r2 = r.row(2);

      Point n0 = r0.cross(r1);
      Point n1 = r0.cross(r2);
      Point n2 = r1.cross(r2);

      float d0 = n0.squaredNorm();
      float d1 = n1.squaredNorm();
      float d2 = n2.squaredNorm();

      float max_d = d0;
      normal = n0;
      if (d1 > max_d) { max_d = d1; normal = n1; }
      if (d2 > max_d) { max_d = d2; normal = n2; }

      if (max_d <= std::numeric_limits<float>::min()) {
        normal = Point::UnitX();  // Fallback
      } else {
        normal.normalize();
      }
    }

    // 4. Orient normal towards viewpoint (assume origin (0,0,0))
    if (normal.dot(-query_pt) < 0) {
      normal = -normal;
    }

    normals[i] = normal;
  }
}

inline void estimateNormals(const PointCloud& cloud,
                            std::vector<Point>& normals,
                            const NormalEstimationConfig& config) {
  search::VoxelHashMap searcher(config.radius);
  searcher.setInputCloud(cloud);

  estimateNormals(cloud, searcher, normals, config);
}

}  // namespace geometry
}  // namespace nanopcl

#endif  // NANOPCL_GEOMETRY_IMPL_NORMAL_ESTIMATION_IMPL_HPP
