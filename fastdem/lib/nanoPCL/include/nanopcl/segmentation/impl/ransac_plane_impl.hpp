// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// RANSAC plane segmentation implementation.
// Do not include this file directly; include
// <nanopcl/segmentation/ransac_plane.hpp>

#ifndef NANOPCL_SEGMENTATION_IMPL_RANSAC_PLANE_IMPL_HPP
#define NANOPCL_SEGMENTATION_IMPL_RANSAC_PLANE_IMPL_HPP

#include <algorithm>
#include <cmath>

namespace nanopcl {
namespace segmentation {

// =============================================================================
// Implementation Details
// =============================================================================

namespace detail {

/**
 * @brief Fast XorShift64 random number generator
 *
 * Much faster than std::mt19937 for RANSAC sampling.
 */
class XorShift64 {
public:
  explicit XorShift64(uint64_t seed = 42)
      : state_(seed ? seed : 1) {}

  uint64_t operator()() noexcept {
    state_ ^= state_ << 13;
    state_ ^= state_ >> 7;
    state_ ^= state_ << 17;
    return state_;
  }

  /// Generate random index in range [0, n)
  size_t randIndex(size_t n) noexcept {
    return static_cast<size_t>((*this)() % n);
  }

private:
  uint64_t state_;
};

/**
 * @brief Compute adaptive iteration count using RANSAC formula
 *
 * k = log(1 - p) / log(1 - w^n)
 * where p = desired probability, w = inlier ratio, n = sample size (3)
 */
inline int computeAdaptiveIterations(double inlier_ratio, double probability) {
  if (inlier_ratio <= 0.0 || inlier_ratio >= 1.0) {
    return std::numeric_limits<int>::max();
  }

  // w^3 for 3-point plane fitting
  double w_cubed = inlier_ratio * inlier_ratio * inlier_ratio;
  double log_prob = std::log(1.0 - probability);
  double log_outlier = std::log(1.0 - w_cubed);

  if (log_outlier >= 0.0) {
    return std::numeric_limits<int>::max();
  }

  return static_cast<int>(std::ceil(log_prob / log_outlier));
}

/**
 * @brief Count inliers for a plane model
 *
 * Single-threaded implementation - benchmarks show this is faster and more
 * stable than OpenMP for typical point cloud sizes (<500K points).
 */
inline size_t countInliers(const PointCloud& cloud,
                           const std::vector<uint32_t>& indices,
                           const PlaneModel& model,
                           float threshold) {
  const float nx = model.coefficients[0];
  const float ny = model.coefficients[1];
  const float nz = model.coefficients[2];
  const float d = model.coefficients[3];

  size_t count = 0;
  for (uint32_t idx : indices) {
    const auto p = cloud.point(idx);
    float dist = std::abs(nx * p.x() + ny * p.y() + nz * p.z() + d);
    if (dist < threshold)
      ++count;
  }

  return count;
}

/**
 * @brief Collect inlier indices for final result
 */
inline std::vector<uint32_t> collectInliers(
    const PointCloud& cloud, const std::vector<uint32_t>& indices, const PlaneModel& model, float threshold) {
  const float nx = model.coefficients[0];
  const float ny = model.coefficients[1];
  const float nz = model.coefficients[2];
  const float d = model.coefficients[3];

  std::vector<uint32_t> inliers;
  inliers.reserve(indices.size() / 2);

  for (uint32_t idx : indices) {
    const auto p = cloud.point(idx);
    float dist = std::abs(nx * p.x() + ny * p.y() + nz * p.z() + d);
    if (dist < threshold) {
      inliers.push_back(idx);
    }
  }

  return inliers;
}

/**
 * @brief Refine plane model using all inliers (Least Squares via PCA)
 *
 * Computes the optimal plane through all inlier points using PCA.
 * The plane normal is the eigenvector corresponding to the smallest eigenvalue
 * of the covariance matrix.
 *
 * @param cloud Point cloud
 * @param inliers Inlier indices
 * @return Refined plane model
 */
inline PlaneModel refinePlane(const PointCloud& cloud,
                              const std::vector<uint32_t>& inliers) {
  if (inliers.size() < 3) {
    return {};
  }

  const size_t n = inliers.size();

  // Step 1: Compute centroid
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (uint32_t idx : inliers) {
    const auto p = cloud.point(idx);
    centroid += Eigen::Vector3d(p.x(), p.y(), p.z());
  }
  centroid /= static_cast<double>(n);

  // Step 2: Compute covariance matrix (only upper triangle needed)
  // C = sum((p - centroid) * (p - centroid)^T)
  double c00 = 0, c01 = 0, c02 = 0;
  double c11 = 0, c12 = 0, c22 = 0;

  for (uint32_t idx : inliers) {
    const auto p = cloud.point(idx);
    double dx = p.x() - centroid.x();
    double dy = p.y() - centroid.y();
    double dz = p.z() - centroid.z();
    c00 += dx * dx;
    c01 += dx * dy;
    c02 += dx * dz;
    c11 += dy * dy;
    c12 += dy * dz;
    c22 += dz * dz;
  }

  // Build symmetric covariance matrix
  Eigen::Matrix3d cov;
  cov << c00, c01, c02, c01, c11, c12, c02, c12, c22;

  // Step 3: Eigen decomposition - smallest eigenvalue's eigenvector is normal
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  if (solver.info() != Eigen::Success) {
    return {};
  }

  // Eigenvalues are sorted in ascending order, so index 0 is smallest
  Eigen::Vector3f normal = solver.eigenvectors().col(0).cast<float>();
  normal.normalize();

  // Step 4: Compute d = -n · centroid
  float d = -normal.dot(centroid.cast<float>());

  PlaneModel result;
  result.coefficients = Eigen::Vector4f(normal.x(), normal.y(), normal.z(), d);
  return result;
}

} // namespace detail

// =============================================================================
// PlaneModel Implementation
// =============================================================================

inline PlaneModel PlaneModel::fromPoints(const Point& p1, const Point& p2, const Point& p3) {
  Eigen::Vector3f v1 = p2 - p1;
  Eigen::Vector3f v2 = p3 - p1;
  Eigen::Vector3f n = v1.cross(v2);

  float norm = n.norm();
  if (norm < 1e-10f) {
    // Collinear points - return invalid plane
    PlaneModel invalid;
    invalid.coefficients = Eigen::Vector4f(0, 0, 0, std::numeric_limits<float>::quiet_NaN());
    return invalid;
  }

  n /= norm; // Normalize
  float d = -n.dot(p1);

  PlaneModel result;
  result.coefficients = Eigen::Vector4f(n.x(), n.y(), n.z(), d);
  return result;
}

// =============================================================================
// RansacResult Implementation
// =============================================================================

inline std::vector<uint32_t> RansacResult::outliers(size_t total_points) const {
  std::vector<bool> is_inlier(total_points, false);
  for (uint32_t idx : inliers) {
    is_inlier[idx] = true;
  }

  std::vector<uint32_t> result;
  result.reserve(total_points - inliers.size());
  for (size_t i = 0; i < total_points; ++i) {
    if (!is_inlier[i]) {
      result.push_back(static_cast<uint32_t>(i));
    }
  }
  return result;
}

// =============================================================================
// segmentPlane Implementation (with indices)
// =============================================================================

inline RansacResult segmentPlane(const PointCloud& cloud,
                                 const std::vector<uint32_t>& indices,
                                 const RansacConfig& config) {
  const size_t n = indices.size();

  // Need at least 3 points to fit a plane
  if (n < 3) {
    return {};
  }

  detail::XorShift64 rng(42);

  PlaneModel best_model;
  size_t best_inlier_count = 0;
  int adaptive_iterations = config.max_iterations;
  int actual_iterations = 0;

  for (int iter = 0; iter < config.max_iterations && iter < adaptive_iterations;
       ++iter) {
    ++actual_iterations;

    // =========================================================================
    // Step 1: Random sample 3 points
    // =========================================================================
    size_t i1 = rng.randIndex(n);
    size_t i2 = rng.randIndex(n);
    size_t i3 = rng.randIndex(n);

    // Ensure distinct indices
    while (i2 == i1)
      i2 = rng.randIndex(n);
    while (i3 == i1 || i3 == i2)
      i3 = rng.randIndex(n);

    const Point p1 = cloud.point(indices[i1]);
    const Point p2 = cloud.point(indices[i2]);
    const Point p3 = cloud.point(indices[i3]);

    // =========================================================================
    // Step 2: Fit plane model
    // =========================================================================
    PlaneModel model = PlaneModel::fromPoints(p1, p2, p3);
    if (!model.isValid()) {
      continue; // Collinear points
    }

    // =========================================================================
    // Step 3: Count inliers (OpenMP parallelized for large clouds)
    // =========================================================================
    size_t inlier_count =
        detail::countInliers(cloud, indices, model, config.distance_threshold);

    // =========================================================================
    // Step 4: Update best model
    // =========================================================================
    if (inlier_count > best_inlier_count) {
      best_model = model;
      best_inlier_count = inlier_count;

      // Update adaptive iteration count
      double inlier_ratio = static_cast<double>(inlier_count) / n;
      adaptive_iterations =
          detail::computeAdaptiveIterations(inlier_ratio, config.probability);
    }
  }

  // ===========================================================================
  // Step 5: Collect inliers for best model
  // ===========================================================================
  if (best_inlier_count < config.min_inliers) {
    return {};
  }

  RansacResult result;
  result.model = best_model;
  result.inliers = detail::collectInliers(cloud, indices, best_model, config.distance_threshold);

  // ===========================================================================
  // Step 6: Refine model using all inliers (Least Squares) + Re-scoring
  // ===========================================================================
  if (config.refine_model && result.inliers.size() >= 3) {
    PlaneModel refined = detail::refinePlane(cloud, result.inliers);
    if (refined.isValid()) {
      result.model = refined;
      // Re-collect inliers with refined model for consistency
      result.inliers = detail::collectInliers(cloud, indices, refined, config.distance_threshold);
    }
  }

  result.fitness = static_cast<double>(result.inliers.size()) / n;
  result.iterations = actual_iterations;

  return result;
}

// =============================================================================
// segmentPlane Implementation (full cloud)
// =============================================================================

inline RansacResult segmentPlane(const PointCloud& cloud,
                                 float distance_threshold,
                                 int max_iterations,
                                 double probability) {
  const size_t n = cloud.size();
  if (n < 3)
    return {};

  // Create index vector for full cloud
  std::vector<uint32_t> indices(n);
  for (size_t i = 0; i < n; ++i) {
    indices[i] = static_cast<uint32_t>(i);
  }

  RansacConfig config;
  config.distance_threshold = distance_threshold;
  config.max_iterations = max_iterations;
  config.probability = probability;

  return segmentPlane(cloud, indices, config);
}

// =============================================================================
// segmentMultiplePlanes Implementation
// =============================================================================

inline std::vector<RansacResult> segmentMultiplePlanes(
    const PointCloud& cloud, const RansacConfig& config, size_t max_planes, double min_fitness) {
  std::vector<RansacResult> results;
  results.reserve(max_planes);

  // Start with all indices
  const size_t n = cloud.size();
  std::vector<uint32_t> remaining(n);
  for (size_t i = 0; i < n; ++i) {
    remaining[i] = static_cast<uint32_t>(i);
  }

  for (size_t plane_idx = 0; plane_idx < max_planes && remaining.size() >= 3;
       ++plane_idx) {
    RansacResult result = segmentPlane(cloud, remaining, config);

    if (!result.success() || result.fitness < min_fitness) {
      break;
    }

    // Remove inliers from remaining points
    std::vector<bool> is_inlier(n, false);
    for (uint32_t idx : result.inliers) {
      is_inlier[idx] = true;
    }

    std::vector<uint32_t> new_remaining;
    new_remaining.reserve(remaining.size() - result.inliers.size());
    for (uint32_t idx : remaining) {
      if (!is_inlier[idx]) {
        new_remaining.push_back(idx);
      }
    }
    remaining = std::move(new_remaining);

    results.push_back(std::move(result));
  }

  return results;
}

} // namespace segmentation
} // namespace nanopcl

#endif // NANOPCL_SEGMENTATION_IMPL_RANSAC_PLANE_IMPL_HPP
