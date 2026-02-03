// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Point-to-Plane ICP factor.

#ifndef NANOPCL_REGISTRATION_FACTORS_PLANE_FACTOR_HPP
#define NANOPCL_REGISTRATION_FACTORS_PLANE_FACTOR_HPP

#include <Eigen/Core>
#include <cmath>

#include "nanopcl/registration/context.hpp"
#include "nanopcl/registration/factors/robust_kernels.hpp"

namespace nanopcl {
namespace registration {

/// @brief Point-to-Plane ICP factor
///
/// Minimizes point-to-plane distance:
///   e = 0.5 * (n^T * (p_src - p_tgt))^2
///
/// where n is the target surface normal.
struct PlaneICPFactor {
  /// Factor settings
  struct Setting {
    RobustKernel robust_kernel = RobustKernel::NONE;
    double robust_kernel_width = 1.0;
  };

  /// @brief Linearize Point-to-Plane cost for one correspondence
  ///
  /// Computes H += J^T * w * J, b += J^T * w * r, e += 0.5 * w * r^2
  ///
  /// @param ctx Plane context containing transformed source, target point, and normal
  /// @param setting Factor settings (robust kernel, etc.)
  /// @param H_out [out] 21-element upper-triangular Hessian contribution
  /// @param b_out [out] 6-element gradient contribution
  /// @param e_out [out] Error contribution
  static void linearize(const PlaneContext& ctx,
                        const Setting& setting,
                        double* H_out,
                        double* b_out,
                        double* e_out) {
    // Normal (assumed unit vector)
    const double nx = ctx.n_tgt.x();
    const double ny = ctx.n_tgt.y();
    const double nz = ctx.n_tgt.z();

    // Point-to-plane residual: r = n^T * (p_src - p_tgt)
    const double r = nx * (ctx.p_src.x() - ctx.p_tgt.x()) +
                     ny * (ctx.p_src.y() - ctx.p_tgt.y()) +
                     nz * (ctx.p_src.z() - ctx.p_tgt.z());

    // Robust weight
    const double w = computeRobustWeight(std::abs(r), setting.robust_kernel,
                                         setting.robust_kernel_width);

    // Jacobian: J = [n^T, (p_src × n)^T] (1x6)
    const double tx = ctx.p_src.x();
    const double ty = ctx.p_src.y();
    const double tz = ctx.p_src.z();

    const double j3 = ty * nz - tz * ny;  // (p_src × n)_x
    const double j4 = tz * nx - tx * nz;  // (p_src × n)_y
    const double j5 = tx * ny - ty * nx;  // (p_src × n)_z

    // H = J^T * w * J (6x6 outer product)
    // H_11 block (n * n^T)
    H_out[0] += w * nx * nx;
    H_out[1] += w * nx * ny;
    H_out[2] += w * nx * nz;
    H_out[6] += w * ny * ny;
    H_out[7] += w * ny * nz;
    H_out[11] += w * nz * nz;

    // H_12 block (n * (p_src × n)^T)
    H_out[3] += w * nx * j3;
    H_out[4] += w * nx * j4;
    H_out[5] += w * nx * j5;
    H_out[8] += w * ny * j3;
    H_out[9] += w * ny * j4;
    H_out[10] += w * ny * j5;
    H_out[12] += w * nz * j3;
    H_out[13] += w * nz * j4;
    H_out[14] += w * nz * j5;

    // H_22 block ((p_src × n) * (p_src × n)^T)
    H_out[15] += w * j3 * j3;
    H_out[16] += w * j3 * j4;
    H_out[17] += w * j3 * j5;
    H_out[18] += w * j4 * j4;
    H_out[19] += w * j4 * j5;
    H_out[20] += w * j5 * j5;

    // b = J^T * w * r
    b_out[0] += w * r * nx;
    b_out[1] += w * r * ny;
    b_out[2] += w * r * nz;
    b_out[3] += w * r * j3;
    b_out[4] += w * r * j4;
    b_out[5] += w * r * j5;

    // Error: 0.5 * w * r^2
    *e_out += 0.5 * w * r * r;
  }
};

}  // namespace registration
}  // namespace nanopcl

#endif  // NANOPCL_REGISTRATION_FACTORS_PLANE_FACTOR_HPP
