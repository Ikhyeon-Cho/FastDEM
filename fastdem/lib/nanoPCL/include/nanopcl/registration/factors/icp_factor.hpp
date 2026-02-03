// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// Point-to-Point ICP factor.

#ifndef NANOPCL_REGISTRATION_FACTORS_ICP_FACTOR_HPP
#define NANOPCL_REGISTRATION_FACTORS_ICP_FACTOR_HPP

#include <Eigen/Core>

#include "nanopcl/registration/context.hpp"

namespace nanopcl {
namespace registration {

/// @brief Point-to-Point ICP factor
///
/// Minimizes point-to-point Euclidean distance:
///   e = 0.5 * ||p_src - p_tgt||^2
///
/// Jacobian: J = [I, -[p_src]×] where [.]× is skew-symmetric matrix
struct ICPFactor {
  /// Factor settings (empty for basic ICP)
  struct Setting {};

  /// @brief Linearize ICP cost for one correspondence
  ///
  /// Computes H += J^T * J, b += J^T * r, e += 0.5 * r^T * r
  ///
  /// @param ctx Point context containing transformed source and target points
  /// @param setting Factor settings (unused for ICP)
  /// @param H_out [out] 21-element upper-triangular Hessian contribution
  /// @param b_out [out] 6-element gradient contribution
  /// @param e_out [out] Error contribution
  static void linearize(const PointContext& ctx,
                        const Setting& /*setting*/,
                        double* H_out,
                        double* b_out,
                        double* e_out) {
    // Residual: r = p_src - p_tgt
    const double rx = ctx.p_src.x() - ctx.p_tgt.x();
    const double ry = ctx.p_src.y() - ctx.p_tgt.y();
    const double rz = ctx.p_src.z() - ctx.p_tgt.z();

    // Transformed point coordinates for Jacobian
    const double tx = ctx.p_src.x();
    const double ty = ctx.p_src.y();
    const double tz = ctx.p_src.z();

    // Jacobian: J = [I, -[p_src]×]
    // J^T * r gives gradient contribution
    // b = [r; p_src × r]
    const double b3 = ty * rz - tz * ry;
    const double b4 = tz * rx - tx * rz;
    const double b5 = tx * ry - ty * rx;

    // H = J^T * J
    // H_11 = I (3x3 identity)
    // H_12 = -[p_src]× (3x3)
    // H_22 = [p_src]× * [p_src]×^T (symmetric 3x3)

    // H_12 = -[p_src]×
    const double h03 = -tz;
    const double h04 = ty;
    const double h05 = 0;
    const double h13 = tz;
    const double h14 = 0;
    const double h15 = -tx;
    const double h23 = -ty;
    const double h24 = tx;
    const double h25 = 0;

    // H_22 = [p_src]× * [p_src]×^T
    const double h33 = ty * ty + tz * tz;
    const double h34 = -tx * ty;
    const double h35 = -tx * tz;
    const double h44 = tx * tx + tz * tz;
    const double h45 = -ty * tz;
    const double h55 = tx * tx + ty * ty;

    // Accumulate upper-triangular H (21 elements)
    // H_11 block (identity)
    H_out[0] += 1.0;
    H_out[1] += 0.0;
    H_out[2] += 0.0;
    H_out[6] += 1.0;
    H_out[7] += 0.0;
    H_out[11] += 1.0;

    // H_12 block
    H_out[3] += h03;
    H_out[4] += h04;
    H_out[5] += h05;
    H_out[8] += h13;
    H_out[9] += h14;
    H_out[10] += h15;
    H_out[12] += h23;
    H_out[13] += h24;
    H_out[14] += h25;

    // H_22 block
    H_out[15] += h33;
    H_out[16] += h34;
    H_out[17] += h35;
    H_out[18] += h44;
    H_out[19] += h45;
    H_out[20] += h55;

    // Accumulate gradient
    b_out[0] += rx;
    b_out[1] += ry;
    b_out[2] += rz;
    b_out[3] += b3;
    b_out[4] += b4;
    b_out[5] += b5;

    // Error: 0.5 * ||r||^2
    *e_out += 0.5 * (rx * rx + ry * ry + rz * rz);
  }
};

}  // namespace registration
}  // namespace nanopcl

#endif  // NANOPCL_REGISTRATION_FACTORS_ICP_FACTOR_HPP
