// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// PCA (Principal Component Analysis) utilities.
//
// Provides eigendecomposition of 3x3 covariance matrices, useful for
// surface normal estimation, curvature analysis, and feature extraction.

#ifndef NANOPCL_GEOMETRY_PCA_HPP
#define NANOPCL_GEOMETRY_PCA_HPP

#include "nanopcl/geometry/impl/pca.hpp"

namespace nanopcl {
namespace geometry {

using detail::PCAResult;
using detail::computePCA;

}  // namespace geometry
}  // namespace nanopcl

#endif  // NANOPCL_GEOMETRY_PCA_HPP
