// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * inpainting.hpp
 *
 * Inpainting configuration for hole filling.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FASTDEM_CONFIG_INPAINTING_HPP
#define FASTDEM_CONFIG_INPAINTING_HPP

namespace fastdem::config {

/**
 * @brief Inpainting configuration for hole filling.
 */
struct Inpainting {
  bool enabled = false;
  int max_iterations = 3;
  int min_valid_neighbors = 2;
};

}  // namespace fastdem::config

#endif  // FASTDEM_CONFIG_INPAINTING_HPP
