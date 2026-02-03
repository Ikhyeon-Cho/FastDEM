// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * timer.hpp
 *
 * Simple timer utility for examples.
 */

#ifndef EXAMPLES_COMMON_TIMER_HPP
#define EXAMPLES_COMMON_TIMER_HPP

#include <chrono>
#include <iostream>
#include <string>

namespace examples {

class Timer {
 public:
  using Clock = std::chrono::high_resolution_clock;

  void start() { start_ = Clock::now(); }

  double elapsedMs() const {
    auto end = Clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }

  double elapsedUs() const {
    auto end = Clock::now();
    return std::chrono::duration<double, std::micro>(end - start_).count();
  }

  void printElapsed(const std::string& label) const {
    double ms = elapsedMs();
    if (ms < 1.0) {
      std::cout << "[" << label << "] " << elapsedUs() << " us" << std::endl;
    } else {
      std::cout << "[" << label << "] " << ms << " ms" << std::endl;
    }
  }

 private:
  Clock::time_point start_;
};

}  // namespace examples

#endif  // EXAMPLES_COMMON_TIMER_HPP
