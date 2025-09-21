/*
 * config.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef PIPELINE_CORE_CONFIG_H
#define PIPELINE_CORE_CONFIG_H

#include <map>
#include <string>
#include <vector>

namespace pipeline {

// Configuration for a single stage
struct StageConfig {
  std::string type;                          // Stage type name (required)
  std::string name;                          // Optional custom name for identification
  bool enabled = true;                       // Stage enabled/disabled flag
  std::map<std::string, std::string> params; // Stage-specific parameters
};

// Configuration for entire pipeline
struct PipelineConfig {
  std::vector<StageConfig> stages;                // List of stages to create
  bool stop_on_error = true;                      // Pipeline error handling behavior
  std::map<std::string, std::string> global_params; // Global parameters for all stages
};

} // namespace pipeline

#endif // PIPELINE_CORE_CONFIG_H