/*
 * inpainting.h
 *
 * Inpainting configuration for hole filling.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CONFIG_INPAINTING_H
#define HEIGHT_MAPPING_CONFIG_INPAINTING_H

namespace height_mapping::config {

/**
 * @brief Inpainting configuration for hole filling.
 */
struct Inpainting {
  bool enabled = false;
  int max_iterations = 3;
  int min_valid_neighbors = 2;
};

}  // namespace height_mapping::config

#endif  // HEIGHT_MAPPING_CONFIG_INPAINTING_H
