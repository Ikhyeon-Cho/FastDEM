// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

#ifndef BENCHMARK_PROFILER_HPP
#define BENCHMARK_PROFILER_HPP

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <numeric>
#include <string>
#include <vector>

#include <spdlog/spdlog.h>

namespace bm {

class Profiler {
 public:
  class ScopedTimer {
   public:
    ScopedTimer(Profiler& profiler, size_t index)
        : profiler_(profiler),
          index_(index),
          start_(std::chrono::steady_clock::now()) {}

    ~ScopedTimer() {
      if (index_ == SIZE_MAX) return;
      double ms = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - start_)
                      .count();
      profiler_.record(index_, ms);
    }

    ScopedTimer(ScopedTimer&& o) noexcept
        : profiler_(o.profiler_), index_(o.index_), start_(o.start_) {
      o.index_ = SIZE_MAX;
    }

    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;
    ScopedTimer& operator=(ScopedTimer&&) = delete;

   private:
    Profiler& profiler_;
    size_t index_;
    std::chrono::steady_clock::time_point start_;
  };

  explicit Profiler(size_t window_size = 100) : window_size_(window_size) {}

  ScopedTimer scope(const std::string& name) {
    for (size_t i = 0; i < stages_.size(); ++i) {
      if (stages_[i].name == name) return ScopedTimer(*this, i);
    }
    stages_.push_back({name, {}, 0});
    return ScopedTimer(*this, stages_.size() - 1);
  }

  void printTable() const {
    if (stages_.empty()) return;

    double total_avg = 0;
    for (const auto& s : stages_) {
      if (s.name == "Total") {
        total_avg = windowAvg(s);
        break;
      }
    }

    size_t window_frames = 0;
    for (const auto& s : stages_) {
      window_frames = std::max(window_frames, s.window.size());
    }

    char buf[256];
    std::string table = "\n";

    snprintf(buf, sizeof(buf), "[FastDEM Profiler] Window: %zu frames\n",
             window_frames);
    table += buf;
    snprintf(buf, sizeof(buf), "%-24s %8s %9s %8s %8s %9s %6s\n", "Stage",
             "Avg(ms)", "Last(ms)", "Min(ms)", "Max(ms)", "Usage(%)", "Calls");
    table += buf;
    table += std::string(78, '-') + "\n";

    for (const auto& s : stages_) {
      if (s.name == "Total") continue;
      double avg = windowAvg(s);
      double usage = (total_avg > 0) ? (avg / total_avg * 100.0) : 0;
      snprintf(buf, sizeof(buf),
               "%-24s %8.2f %9.2f %8.2f %8.2f %8.1f%% %6zu\n",
               s.name.c_str(), avg, s.last_ms, windowMin(s), windowMax(s),
               usage, s.window.size());
      table += buf;
    }

    for (const auto& s : stages_) {
      if (s.name != "Total") continue;
      table += std::string(78, '-') + "\n";
      snprintf(buf, sizeof(buf),
               "%-24s %8.2f %9.2f %8.2f %8.2f %8.1f%% %6zu\n",
               s.name.c_str(), windowAvg(s), s.last_ms, windowMin(s),
               windowMax(s), 100.0, s.window.size());
      table += buf;
    }

    spdlog::info(table);
  }

  size_t windowSize() const { return window_size_; }
  void setWindowSize(size_t size) { window_size_ = size; }

 private:
  struct StageData {
    std::string name;
    std::deque<double> window;
    double last_ms = 0;
  };

  static double windowAvg(const StageData& s) {
    if (s.window.empty()) return 0;
    return std::accumulate(s.window.begin(), s.window.end(), 0.0) /
           static_cast<double>(s.window.size());
  }

  static double windowMin(const StageData& s) {
    if (s.window.empty()) return 0;
    return *std::min_element(s.window.begin(), s.window.end());
  }

  static double windowMax(const StageData& s) {
    if (s.window.empty()) return 0;
    return *std::max_element(s.window.begin(), s.window.end());
  }

  void record(size_t index, double ms) {
    auto& stage = stages_[index];
    stage.last_ms = ms;
    stage.window.push_back(ms);
    if (stage.window.size() > window_size_) stage.window.pop_front();
  }

  size_t window_size_;
  std::vector<StageData> stages_;
};

}  // namespace bm

#endif  // BENCHMARK_PROFILER_HPP
