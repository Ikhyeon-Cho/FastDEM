/*
 * exceptions.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef PIPELINE_CORE_EXCEPTIONS_H
#define PIPELINE_CORE_EXCEPTIONS_H

#include <stdexcept>
#include <string>

namespace pipeline {

/**
 * @brief Base exception for stage processing errors
 */
class StageError : public std::runtime_error {
public:
  StageError(const std::string& stage_name, const std::string& message)
      : std::runtime_error("[" + stage_name + "] " + message),
        stage_name_(stage_name),
        message_(message) {}

  const std::string& getStage() const { return stage_name_; }
  const std::string& getMessage() const { return message_; }

private:
  std::string stage_name_;
  std::string message_;
};

/**
 * @brief Critical error that requires immediate pipeline termination
 *
 * Examples:
 * - Transform provider destroyed
 * - Map initialization failed
 * - Required resource unavailable
 */
class CriticalError : public StageError {
public:
  CriticalError(const std::string& stage_name, const std::string& message)
      : StageError(stage_name, message) {}
};

/**
 * @brief Recoverable error that may allow pipeline to continue
 *
 * Examples:
 * - Temporary transform unavailable
 * - Invalid data that can be skipped
 * - Non-essential computation failed
 */
class RecoverableError : public StageError {
public:
  RecoverableError(const std::string& stage_name, const std::string& message)
      : StageError(stage_name, message) {}
};

} // namespace pipeline

#endif // PIPELINE_CORE_EXCEPTIONS_H