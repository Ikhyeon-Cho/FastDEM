// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * covariance_util.hpp
 *
 * Utility functions for covariance propagation.
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_SENSORS_COVARIANCE_UTIL_HPP
#define FASTDEM_SENSORS_COVARIANCE_UTIL_HPP

#include <Eigen/Core>
#include <vector>

#include <nanopcl/core/types.hpp>

namespace fastdem::utils {

/**
 * @brief Project 3D covariance to height (Z) variance.
 *
 * σ_z² = r_z^T × Σ × r_z
 *
 * @param cov Covariance matrix in source frame
 * @param R Rotation from source to target frame
 * @return Height variance σ_z² in target frame
 */
inline float projectToHeightVariance(const Eigen::Matrix3f& cov,
                                     const Eigen::Matrix3f& R) {
  Eigen::Vector3f r_z = R.row(2).transpose();
  return r_z.transpose() * cov * r_z;
}

/**
 * @brief Project covariances to height variances (batch).
 *
 * @param covariances Covariance matrices in source frame
 * @param R Rotation from source to target frame
 * @return Height variances σ_z² in target frame
 */
inline std::vector<float> projectToHeightVariances(
    const nanopcl::AlignedVector<nanopcl::Covariance>& covariances,
    const Eigen::Matrix3f& R) {
  std::vector<float> variances;
  variances.reserve(covariances.size());

  for (const auto& cov : covariances) {
    variances.push_back(projectToHeightVariance(cov, R));
  }

  return variances;
}

}  // namespace fastdem::utils

#endif  // FASTDEM_SENSORS_COVARIANCE_UTIL_HPP
