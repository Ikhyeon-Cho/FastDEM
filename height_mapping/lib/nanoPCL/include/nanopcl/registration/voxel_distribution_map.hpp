// nanoPCL - Header-only C++17 point cloud library
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// VoxelDistributionMap: Voxel-based distribution storage for VGICP.
//
// This data structure stores the mean and covariance of points within each
// voxel, enabling O(1) lookup for Voxelized GICP registration.
//
// Key design choices:
//   - Open Addressing hash table (cache-friendly, no linked lists)
//   - Linear Probing for collision resolution
//   - Pre-computed regularized covariances for GICP
//
// Example usage:
//   VoxelDistributionMap voxel_map(0.5f);  // 0.5m resolution
//   voxel_map.build(target_cloud);
//
//   // O(1) lookup
//   auto dist = voxel_map.lookup(query_point);
//   if (dist) {
//     Eigen::Vector3f mean = dist->mean;
//     Eigen::Matrix3f cov = dist->covariance;
//   }

#ifndef NANOPCL_REGISTRATION_VOXEL_DISTRIBUTION_MAP_HPP
#define NANOPCL_REGISTRATION_VOXEL_DISTRIBUTION_MAP_HPP

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/core/voxel_key.hpp"

#ifdef _OPENMP
#include <omp.h>
#endif

namespace npcl {
namespace registration {

/**
 * @brief Distribution statistics for a single voxel
 */
struct VoxelDistribution {
  Eigen::Vector3f mean;        ///< Centroid of points in voxel
  Eigen::Matrix3f covariance;  ///< Covariance matrix of points
  uint32_t num_points;         ///< Number of points in voxel

  /// Check if distribution is valid (has enough points)
  [[nodiscard]] bool isValid() const { return num_points >= 3; }
};

/**
 * @brief Voxel-based distribution map for VGICP
 *
 * Stores per-voxel mean and covariance for efficient plane-to-plane
 * registration. Uses Open Addressing hash table with Linear Probing.
 */
class VoxelDistributionMap {
 public:
  /**
   * @brief Construct with specified voxel resolution
   * @param resolution Voxel size in meters
   * @param covariance_epsilon Regularization for covariance (default: 1e-3)
   */
  explicit VoxelDistributionMap(float resolution = 0.5f,
                                double covariance_epsilon = 1e-3)
      : resolution_(resolution),
        inv_resolution_(1.0f / resolution),
        covariance_epsilon_(covariance_epsilon) {}

  /**
   * @brief Build distribution map from point cloud
   *
   * Computes mean and covariance for each occupied voxel.
   * Parallelized with OpenMP.
   *
   * @param cloud Input point cloud
   */
  void build(const PointCloud& cloud);

  /**
   * @brief Lookup distribution for a query point (O(1) average)
   * @param p Query point in continuous space
   * @return Distribution if voxel exists and is valid, nullopt otherwise
   */
  [[nodiscard]] std::optional<VoxelDistribution> lookup(const Point& p) const;

  /**
   * @brief Lookup with pre-regularized covariance for GICP
   *
   * Returns covariance regularized per Segal et al. (RSS 2009):
   * eigenvalues normalized to [epsilon, 1, 1].
   *
   * @param p Query point
   * @return Distribution with regularized covariance, or nullopt
   */
  [[nodiscard]] std::optional<VoxelDistribution> lookupRegularized(
      const Point& p) const;

  // ===========================================================================
  // Accessors
  // ===========================================================================

  [[nodiscard]] float resolution() const noexcept { return resolution_; }
  [[nodiscard]] float invResolution() const noexcept { return inv_resolution_; }
  [[nodiscard]] size_t numVoxels() const noexcept { return num_valid_voxels_; }
  [[nodiscard]] bool empty() const noexcept { return num_valid_voxels_ == 0; }

 private:
  /// Internal slot for hash table
  struct Slot {
    VoxelKey key;
    VoxelDistribution dist;
    Eigen::Matrix3f regularized_cov;  ///< Pre-computed regularized covariance
    bool occupied = false;
  };

  /// Regularize covariance matrix per Segal et al. (RSS 2009)
  static Eigen::Matrix3f regularizeCovariance(const Eigen::Matrix3f& cov,
                                              double epsilon);

  float resolution_;
  float inv_resolution_;
  double covariance_epsilon_;

  std::vector<Slot> table_;
  size_t mask_ = 0;
  size_t num_valid_voxels_ = 0;
};

// =============================================================================
// Implementation
// =============================================================================

inline void VoxelDistributionMap::build(const PointCloud& cloud) {
  if (cloud.empty()) {
    table_.clear();
    num_valid_voxels_ = 0;
    return;
  }

  const size_t N = cloud.size();

  // ===========================================================================
  // Phase 1: Count points per voxel (parallel-safe accumulation)
  // ===========================================================================

  // Temporary storage: key -> (sum, sum_sq, count)
  struct VoxelAccum {
    VoxelKey key;
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    Eigen::Matrix3d sum_outer = Eigen::Matrix3d::Zero();  // sum of p * p^T
    uint32_t count = 0;
  };

  // Prepare hash table (load factor ~0.5)
  size_t table_size = 1;
  while (table_size < 2 * N) table_size <<= 1;
  mask_ = table_size - 1;

  std::vector<VoxelAccum> accum_table(table_size);

  // Initialize keys as invalid
  for (auto& slot : accum_table) {
    slot.key = VoxelKey::invalid();
  }

  // Sequential accumulation (atomic operations too expensive for this)
  for (size_t i = 0; i < N; ++i) {
    const Point& pt = cloud[i];
    if (!std::isfinite(pt.x()) || !std::isfinite(pt.y()) ||
        !std::isfinite(pt.z())) {
      continue;
    }

    VoxelKey key = VoxelKey::from(pt, inv_resolution_);
    uint64_t hash = key.hash();
    size_t slot = hash & mask_;

    // Linear probing to find or create slot
    while (true) {
      if (!accum_table[slot].key.isValid()) {
        // Empty slot - insert new
        accum_table[slot].key = key;
        accum_table[slot].sum = pt.cast<double>();
        accum_table[slot].sum_outer =
            pt.cast<double>() * pt.cast<double>().transpose();
        accum_table[slot].count = 1;
        break;
      } else if (accum_table[slot].key == key) {
        // Existing slot - accumulate
        Eigen::Vector3d pd = pt.cast<double>();
        accum_table[slot].sum += pd;
        accum_table[slot].sum_outer += pd * pd.transpose();
        accum_table[slot].count++;
        break;
      }
      slot = (slot + 1) & mask_;
    }
  }

  // ===========================================================================
  // Phase 2: Compute mean and covariance from accumulated values
  // ===========================================================================

  table_.resize(table_size);
  num_valid_voxels_ = 0;

  // Can be parallelized since each slot is independent
#ifdef _OPENMP
#pragma omp parallel for reduction(+ : num_valid_voxels_)
#endif
  for (size_t i = 0; i < table_size; ++i) {
    const auto& acc = accum_table[i];

    if (!acc.key.isValid() || acc.count < 3) {
      table_[i].occupied = false;
      table_[i].key = VoxelKey::invalid();
      continue;
    }

    // Compute mean
    Eigen::Vector3d mean = acc.sum / static_cast<double>(acc.count);

    // Compute covariance: E[XX^T] - E[X]E[X]^T
    Eigen::Matrix3d cov = acc.sum_outer / static_cast<double>(acc.count) -
                          mean * mean.transpose();

    // Store in table
    table_[i].key = acc.key;
    table_[i].dist.mean = mean.cast<float>();
    table_[i].dist.covariance = cov.cast<float>();
    table_[i].dist.num_points = acc.count;
    table_[i].regularized_cov =
        regularizeCovariance(table_[i].dist.covariance, covariance_epsilon_);
    table_[i].occupied = true;

    num_valid_voxels_++;
  }
}

inline std::optional<VoxelDistribution> VoxelDistributionMap::lookup(
    const Point& p) const {
  if (table_.empty()) return std::nullopt;

  VoxelKey key = VoxelKey::from(p, inv_resolution_);
  uint64_t hash = key.hash();
  size_t slot = hash & mask_;

  // Linear probing search
  while (table_[slot].occupied) {
    if (table_[slot].key == key) {
      return table_[slot].dist;
    }
    slot = (slot + 1) & mask_;
  }

  return std::nullopt;
}

inline std::optional<VoxelDistribution> VoxelDistributionMap::lookupRegularized(
    const Point& p) const {
  if (table_.empty()) return std::nullopt;

  VoxelKey key = VoxelKey::from(p, inv_resolution_);
  uint64_t hash = key.hash();
  size_t slot = hash & mask_;

  // Linear probing search
  while (table_[slot].occupied) {
    if (table_[slot].key == key) {
      VoxelDistribution result = table_[slot].dist;
      result.covariance = table_[slot].regularized_cov;
      return result;
    }
    slot = (slot + 1) & mask_;
  }

  return std::nullopt;
}

inline Eigen::Matrix3f VoxelDistributionMap::regularizeCovariance(
    const Eigen::Matrix3f& cov, double epsilon) {
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);

  // Normalize per Segal et al. (RSS 2009): [epsilon, 1, 1]
  Eigen::Vector3f eigenvalues;
  eigenvalues(0) = static_cast<float>(epsilon);  // Smallest: normal direction
  eigenvalues(1) = 1.0f;                         // Tangent direction 1
  eigenvalues(2) = 1.0f;                         // Tangent direction 2

  return solver.eigenvectors() * eigenvalues.asDiagonal() *
         solver.eigenvectors().transpose();
}

}  // namespace registration
}  // namespace npcl

#endif  // NANOPCL_REGISTRATION_VOXEL_DISTRIBUTION_MAP_HPP
