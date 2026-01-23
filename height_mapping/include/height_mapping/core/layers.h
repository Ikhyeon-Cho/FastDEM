/*
 * layers.h
 *
 * Layer name string constants for HeightMap.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_LAYERS_H
#define HEIGHT_MAPPING_CORE_LAYERS_H

namespace height_mapping::layer {

// Core layers
constexpr auto elevation = "elevation";
constexpr auto elevation_min = "elevation_min";
constexpr auto elevation_max = "elevation_max";
constexpr auto elevation_max_denoised = "elevation_max_denoised";

// Layers for Kalman estimator
constexpr auto state = "state";  // x (Kalman state estimate)
constexpr auto variance = "variance";
constexpr auto upper_bound = "upper_bound";
constexpr auto lower_bound = "lower_bound";
constexpr auto uncertainty_range = "uncertainty_range";  // upper - lower

// Layers for Welford's algorithm
constexpr auto mean = "mean";
constexpr auto sample_count = "sample_count";
constexpr auto standard_error = "standard_error";
constexpr auto conf_interval_95 = "conf_interval_95";

// Layers for P2 quantile estimator (marker heights)
constexpr auto p2_q0 = "p2_q0";  // 1st percentile (lower bound)
constexpr auto p2_q1 = "p2_q1";  // 16th percentile
constexpr auto p2_q2 = "p2_q2";  // 50th percentile (median)
constexpr auto p2_q3 = "p2_q3";  // 84th percentile
constexpr auto p2_q4 = "p2_q4";  // 99th percentile (upper bound)
// Layers for P2 quantile estimator (marker positions)
constexpr auto p2_n0 = "p2_n0";
constexpr auto p2_n1 = "p2_n1";
constexpr auto p2_n2 = "p2_n2";
constexpr auto p2_n3 = "p2_n3";
constexpr auto p2_n4 = "p2_n4";

// Derived statistics
constexpr auto elevation_inpainted = "elevation_inpainted";
constexpr auto conflict_count = "conflict_count";  // Raycasting temporal voting

// Extension layers (add at use site when needed)
constexpr auto intensity = "intensity";
constexpr auto color = "color";  // Packed RGBA for grid_map_rviz_plugin
constexpr auto label = "label";
constexpr auto raycasting_upper_bound = "raycasting_upper_bound";
constexpr auto traversability = "traversability";

}  // namespace height_mapping::layer

#endif  // HEIGHT_MAPPING_CORE_LAYERS_H
