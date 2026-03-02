// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * feature_extraction.cpp
 *
 * PCA-based terrain feature extraction from elevation map.
 * Computes step, slope, roughness, curvature, and surface normals.
 *
 *  Created on: Feb 2026
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fastdem/postprocess/feature_extraction.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <nanopcl/geometry/pca.hpp>

#include "fastdem/elevation_map.hpp"

namespace fastdem {

void applyFeatureExtraction(ElevationMap& map, float analysis_radius,
                            int min_valid_neighbors,
                            float step_lower_percentile,
                            float step_upper_percentile) {
  if (!map.exists(layer::elevation)) return;

  // Ensure output layers exist
  if (!map.exists(layer::step)) map.add(layer::step, NAN);
  if (!map.exists(layer::slope)) map.add(layer::slope, NAN);
  if (!map.exists(layer::roughness)) map.add(layer::roughness, NAN);
  if (!map.exists(layer::curvature)) map.add(layer::curvature, NAN);
  if (!map.exists(layer::normal_x)) map.add(layer::normal_x, NAN);
  if (!map.exists(layer::normal_y)) map.add(layer::normal_y, NAN);
  if (!map.exists(layer::normal_z)) map.add(layer::normal_z, NAN);

  const auto& elev = map.get(layer::elevation);
  auto& step_mat = map.get(layer::step);
  auto& slope_mat = map.get(layer::slope);
  auto& roughness_mat = map.get(layer::roughness);
  auto& curvature_mat = map.get(layer::curvature);
  auto& nx_mat = map.get(layer::normal_x);
  auto& ny_mat = map.get(layer::normal_y);
  auto& nz_mat = map.get(layer::normal_z);

  const auto reg = map.region(analysis_radius);
  const double res = map.getResolution();

  // Reusable buffer for percentile-based step computation
  std::vector<float> z_vals;
  z_vals.reserve(reg.entries.size());

  for (auto cell : map.cells()) {
    const float center_z = elev(cell.index);
    if (!std::isfinite(center_z)) continue;

    // Accumulate neighbors (single pass: covariance + z values)
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();
    Eigen::Matrix3f sum_sq = Eigen::Matrix3f::Zero();
    z_vals.clear();
    int count = 0;

    for (auto n : map.neighbors(cell, reg)) {
      const float nz = elev(n.index);
      if (!std::isfinite(nz)) continue;

      // World-frame displacement (grid_map: row→-x, col→-y)
      const int dr = n.row - cell.row;
      const int dc = n.col - cell.col;
      const Eigen::Vector3f d(-dr * static_cast<float>(res),
                              -dc * static_cast<float>(res), nz - center_z);

      sum += d;
      sum_sq.noalias() += d * d.transpose();
      z_vals.push_back(nz);
      ++count;
    }

    if (count < min_valid_neighbors) continue;

    // Covariance matrix
    const float inv_n = 1.0f / static_cast<float>(count);
    const Eigen::Vector3f mean = sum * inv_n;
    const Eigen::Matrix3f cov = sum_sq * inv_n - mean * mean.transpose();

    // PCA — skip degenerate patches (collinear or insufficient spread)
    constexpr float kMinEigenvalue = 1e-8f;
    const auto pca = nanopcl::geometry::computePCA(cov);
    if (!pca.valid) continue;
    if (pca.eigenvalues(1) < kMinEigenvalue) continue;

    // Normal vector (smallest eigenvector, flipped upward)
    Eigen::Vector3f normal = pca.eigenvectors.col(0);
    if (normal.z() < 0.0f) normal = -normal;

    // Step: percentile-based range (robust to outliers)
    std::sort(z_vals.begin(), z_vals.end());
    const int lo = static_cast<int>(step_lower_percentile * (count - 1));
    const int hi = static_cast<int>(step_upper_percentile * (count - 1));
    step_mat(cell.index) = z_vals[hi] - z_vals[lo];

    slope_mat(cell.index) =
        std::acos(std::abs(normal.z())) * 180.0f / static_cast<float>(M_PI);
    roughness_mat(cell.index) = std::sqrt(pca.eigenvalues(0));
    const float trace = cov.trace();
    curvature_mat(cell.index) =
        (trace > 0.0f) ? std::abs(pca.eigenvalues(0) / trace) : 0.0f;
    nx_mat(cell.index) = normal.x();
    ny_mat(cell.index) = normal.y();
    nz_mat(cell.index) = normal.z();
  }
}

}  // namespace fastdem
