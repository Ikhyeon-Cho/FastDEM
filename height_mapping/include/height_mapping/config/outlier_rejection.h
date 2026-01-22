/*
 * outlier_rejection.h
 *
 * Outlier rejection configuration.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CONFIG_OUTLIER_REJECTION_H
#define HEIGHT_MAPPING_CONFIG_OUTLIER_REJECTION_H

namespace height_mapping::config {

/**
 * @brief Outlier rejection configuration.
 *
 * Rejects points where |z - existing_elevation| > k × σ.
 */
struct OutlierRejection {
  bool enabled = true;
  float sigma_threshold = 3.0f;   ///< k-sigma threshold for rejection
  float min_uncertainty = 0.05f;  ///< [m] minimum σ floor
};

}  // namespace height_mapping::config

#endif  // HEIGHT_MAPPING_CONFIG_OUTLIER_REJECTION_H
