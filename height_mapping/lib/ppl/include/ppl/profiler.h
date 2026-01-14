/*
 * profiler.h - Pipeline profiler for ppl (Pipeline Library)
 *
 * Decorator pattern implementation for measuring stage execution times.
 * Completely decoupled from Pipeline - Pipeline doesn't know about profiling.
 *
 * Usage:
 *   ppl::Pipeline<MyContext> pipeline;
 *   pipeline.load("config.yaml");
 *
 *   ppl::Profiler<MyContext> profiler(pipeline);
 *   profiler.setAutoprint(true);
 *   profiler.setPrintInterval(100);
 *
 *   // Use profiler.run() instead of pipeline.run()
 *   profiler.run(context);
 *
 * IMPORTANT: Profiler holds a reference to Pipeline.
 * Pipeline must outlive Profiler to avoid dangling reference.
 * Typically, both should have the same lifetime scope.
 *
 * Created on: Dec 2024
 * Author: Ikhyeon Cho
 * Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 * Email: tre0430@korea.ac.kr
 */

#ifndef PPL_PROFILER_H
#define PPL_PROFILER_H

#include "detail/log.h"
#include "pipeline.h"

#include <spdlog/fmt/fmt.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace ppl {

/**
 * @brief Profiler for measuring pipeline performance
 *
 * Wraps a Pipeline and measures execution time of each stage.
 * Uses Decorator pattern to add profiling without modifying Pipeline.
 *
 * @note Pipeline must outlive Profiler (dangling reference risk otherwise)
 *
 * @tparam TContext User-defined context type
 */
template <typename TContext>
class Profiler {
public:
  struct Metrics {
    uint64_t last_us = 0;
    uint64_t total_us = 0;
    uint64_t min_us = UINT64_MAX;
    uint64_t max_us = 0;
    size_t call_count = 0;

    double avgMs() const {
      return call_count > 0 ? (total_us / 1000.0) / call_count : 0;
    }
    double lastMs() const { return last_us / 1000.0; }
    double minMs() const { return min_us == UINT64_MAX ? 0 : min_us / 1000.0; }
    double maxMs() const { return max_us / 1000.0; }
  };

  struct Report {
    std::string stage_name;
    double avg_ms;
    double last_ms;
    double min_ms;
    double max_ms;
    double percentage;
    size_t call_count;
  };

  /**
   * @brief Construct profiler for a pipeline
   *
   * @note Pipeline must be kept alive for the lifetime of this Profiler.
   *       Metrics initialization is deferred to first run() call,
   *       allowing Pipeline to be loaded after Profiler construction.
   *
   * @param pipeline Reference to the pipeline to profile
   */
  explicit Profiler(Pipeline<TContext>& pipeline) : pipeline_(pipeline) {}

  /**
   * @brief Run pipeline with profiling
   *
   * Measures execution time of each stage and updates metrics.
   * Use this instead of pipeline.run()
   *
   * @param context Shared pointer to the context (data bus)
   * @return true if all stages succeeded, false if any stage failed
   */
  bool run(const std::shared_ptr<TContext>& context) {
    const auto& stages = pipeline_.stages();

    // Lazy initialization: resize vectors when stage count changes
    // (handles Pipeline loaded after Profiler construction)
    if (stage_names_.size() != stages.size()) {
      stage_names_.resize(stages.size());
      stage_metrics_.resize(stages.size());
      for (size_t i = 0; i < stages.size(); ++i) {
        stage_names_[i] = stages[i]->name();
      }
    }

    for (size_t i = 0; i < stages.size(); ++i) {
      auto& stage = stages[i];

      auto start = std::chrono::high_resolution_clock::now();

      bool success = false;
      try {
        success = stage->process(context);
      } catch (const std::exception& e) {
        // Record time even on exception (for debugging)
        auto end = std::chrono::high_resolution_clock::now();
        auto duration_us =
            std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                .count();
        updateMetrics(i, duration_us);

        PPL_LOG_ERROR("[ppl] Stage '{}' threw exception: {}", stage->name(),
                      e.what());
        return false;
      }

      auto end = std::chrono::high_resolution_clock::now();
      auto duration_us =
          std::chrono::duration_cast<std::chrono::microseconds>(end - start)
              .count();

      updateMetrics(i, duration_us);

      // Optional per-stage logging (console only)
      if (log_each_stage_) {
        fmt::print("[ppl] {}: {:.2f} ms\n", stage->name(),
                   duration_us / 1000.0);
      }

      if (!success) {
        PPL_LOG_WARN("[ppl] Stage '{}' returned false, stopping pipeline",
                     stage->name());
        return false;
      }
    }

    total_runs_++;

    // Auto-print report at intervals
    if (auto_print_ && total_runs_ % print_interval_ == 0) {
      printReport();
      resetWindow();
    }

    return true;
  }

  /**
   * @brief Generate performance report
   * @return Vector of stage reports
   */
  std::vector<Report> getReport() const {
    std::vector<Report> reports;
    uint64_t total_time = 0;

    for (const auto& metrics : stage_metrics_) {
      total_time += metrics.total_us;
    }

    for (size_t i = 0; i < stage_names_.size(); ++i) {
      const auto& metrics = stage_metrics_[i];
      if (metrics.call_count == 0) continue;

      reports.push_back(
          {stage_names_[i], metrics.avgMs(), metrics.lastMs(), metrics.minMs(),
           metrics.maxMs(),
           total_time > 0 ? (metrics.total_us * 100.0 / total_time) : 0,
           metrics.call_count});
    }

    return reports;
  }

  /**
   * @brief Print formatted performance report to console
   *
   * Uses fmt::print for console-only output (not logged to file)
   */
  void printReport() const {
    auto reports = getReport();
    if (reports.empty()) return;

    size_t window_num = (total_runs_ - 1) / print_interval_ + 1;
    size_t window_start = (window_num - 1) * print_interval_ + 1;

    fmt::print(
        "\n========== Pipeline Benchmark (Window {}: Runs {}-{}) "
        "==========\n",
        window_num, window_start, total_runs_);
    fmt::print("{:<25} {:>8} {:>9} {:>8} {:>8} {:>9} {:>7}\n", "Stage",
               "Avg(ms)", "Last(ms)", "Min(ms)", "Max(ms)", "Usage(%)",
               "Calls");
    fmt::print(
        "---------------------------------------------------------------"
        "------------------\n");

    double total_avg = 0;
    for (const auto& r : reports) {
      fmt::print("{:<25} {:>8.2f} {:>9.2f} {:>8.2f} {:>8.2f} {:>8.1f}% {:>7}\n",
                 r.stage_name, r.avg_ms, r.last_ms, r.min_ms, r.max_ms,
                 r.percentage, r.call_count);
      total_avg += r.avg_ms;
    }

    fmt::print(
        "---------------------------------------------------------------"
        "------------------\n");
    double hz = total_avg > 0 ? 1000.0 / total_avg : 0;
    fmt::print("Total Pipeline:           {:>8.2f} ms ({:.0f} Hz)\n\n", total_avg, hz);
  }

  // Configuration
  void setAutoprint(bool enable) { auto_print_ = enable; }
  void setPrintInterval(size_t interval) { print_interval_ = interval; }
  void setLogEachStage(bool enable) { log_each_stage_ = enable; }

  // Reset all metrics
  void reset() {
    stage_names_.clear();
    stage_metrics_.clear();
    total_runs_ = 0;
  }

  // Get total runs count
  size_t totalRuns() const { return total_runs_; }

private:
  void updateMetrics(size_t idx, uint64_t duration_us) {
    auto& m = stage_metrics_[idx];
    m.last_us = duration_us;
    m.total_us += duration_us;
    m.min_us = std::min(m.min_us, duration_us);
    m.max_us = std::max(m.max_us, duration_us);
    m.call_count++;
  }

  void resetWindow() {
    for (auto& m : stage_metrics_) {
      m.total_us = 0;
      m.call_count = 0;
      m.min_us = UINT64_MAX;
      m.max_us = 0;
    }
  }

  Pipeline<TContext>& pipeline_;
  std::vector<std::string> stage_names_;
  std::vector<Metrics> stage_metrics_;
  size_t total_runs_ = 0;

  // Configuration
  bool auto_print_ = false;
  size_t print_interval_ = 100;
  bool log_each_stage_ = false;
};

}  // namespace ppl

#endif  // PPL_PROFILER_H
