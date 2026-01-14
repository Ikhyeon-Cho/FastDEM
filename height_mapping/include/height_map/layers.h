/*
 * layers.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_LAYERS_H
#define HEIGHT_MAP_LAYERS_H

namespace height_map {
namespace layer {

// Core layers
constexpr auto elevation = "elevation";
constexpr auto variance = "variance";
constexpr auto elevation_min = "elevation_min";
constexpr auto elevation_max = "elevation_max";
constexpr auto count = "measurement_count";
constexpr auto persistence = "persistence";  // Obstacle durability

// Derived statistics
constexpr auto standard_error = "standard_error";
constexpr auto confidence_interval = "confidence_interval_95";
constexpr auto elevation_inpainted = "elevation_inpainted";

// Extension layers (add at use site when needed)
constexpr auto intensity = "intensity";
constexpr auto r = "r";
constexpr auto g = "g";
constexpr auto b = "b";
constexpr auto label = "label";
constexpr auto ray_height = "ray_height";
constexpr auto traversability = "traversability";

}  // namespace layer
}  // namespace height_map

#endif  // HEIGHT_MAP_LAYERS_H
