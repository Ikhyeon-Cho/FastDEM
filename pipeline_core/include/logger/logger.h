/*
 * logger.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef LOGGER_LOGGER_H
#define LOGGER_LOGGER_H

#include <iostream>
#include <string>

namespace pipeline {

class Logger {
public:
  static Logger& instance() {
    static Logger instance;
    return instance;
  }

  void setVerbose(bool verbose) {
    verbose_ = verbose;
  }

  void log(const std::string& msg) {
    if (verbose_) {
      std::cout << msg << std::endl;
    }
  }

private:
  Logger() = default;
  bool verbose_ = false;
};

// Simple macro for logging
#define LOG(msg) pipeline::Logger::instance().log(msg)

} // namespace pipeline

#endif // LOGGER_LOGGER_H