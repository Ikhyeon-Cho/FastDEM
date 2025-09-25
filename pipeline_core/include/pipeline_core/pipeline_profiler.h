/*
 * pipeline_profiler.h
 *
 * Pipeline profiling and benchmarking utility.
 *
 * This class provides non-invasive profiling for pipeline execution,
 * measuring stage execution times and generating performance reports.
 * It follows the External Profiler pattern to maintain separation of concerns.
 *
 * Usage:
 *   auto profiler = std::make_unique<PipelineProfiler>(&pipeline);
 *   profiler->setAutoprint(true);
 *   profiler->setPrintInterval(100);
 *
 *   // Process with profiling
 *   profiler->processWithProfiling(context);
 *
 * Created on: Dec 2024
 * Author: Ikhyeon Cho
 * Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 * Email: tre0430@korea.ac.kr
 */

#ifndef PIPELINE_CORE_PIPELINE_PROFILER_H
#define PIPELINE_CORE_PIPELINE_PROFILER_H

#include "logger/logger.h"
#include "pipeline_core/context.h"
#include "pipeline_core/pipeline.h"
#include <chrono>
#include <map>
#include <string>
#include <vector>

namespace pipeline {

class PipelineProfiler {
public:
  struct Metrics {
    uint64_t last_duration_us = 0;
    uint64_t total_duration_us = 0;
    uint64_t min_duration_us = UINT64_MAX;
    uint64_t max_duration_us = 0;
    size_t call_count = 0;

    double getAvgMs() const {
      return call_count > 0 ? (total_duration_us / 1000.0) / call_count : 0;
    }
    double getLastMs() const { return last_duration_us / 1000.0; }
    double getMinMs() const {
      return min_duration_us == UINT64_MAX ? 0 : min_duration_us / 1000.0;
    }
    double getMaxMs() const { return max_duration_us / 1000.0; }
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

  explicit PipelineProfiler(Pipeline *pipeline) : pipeline_(pipeline) {}

  // Process with profiling - measures each stage
  void processWithProfiling(Context &ctx) {
    // Get stages reference once to avoid multiple calls
    const auto &stages = pipeline_->getStages();

    for (size_t i = 0; i < stages.size(); ++i) {
      auto &stage = stages[i];
      if (!stage->isEnabled() || !stage->canProcess(ctx)) {
        continue;
      }

      const std::string &stage_name = stage->getName();

      auto start = std::chrono::high_resolution_clock::now();
      stage->process(ctx);
      auto end = std::chrono::high_resolution_clock::now();

      auto duration_us =
          std::chrono::duration_cast<std::chrono::microseconds>(end - start)
              .count();
      updateMetrics(i, stage_name, duration_us);

      // Optional per-stage logging
      if (log_each_stage_) {
        LOG_BENCH(stage_name, duration_us / 1000.0);
      }
    }

    total_runs_++;

    // Auto-print if enabled
    if (auto_print_ && total_runs_ % print_interval_ == 0) {
      printReport();
      // Reset metrics after printing for rolling window
      stage_names_.clear();
      stage_metrics_.clear();
    }
  }

  // Generate report
  std::vector<Report> getReport() const {
    std::vector<Report> reports;
    uint64_t total_time = 0;

    // Calculate total time
    for (const auto &metrics : stage_metrics_) {
      total_time += metrics.total_duration_us;
    }

    // Generate individual reports in pipeline order
    for (size_t i = 0; i < stage_names_.size(); ++i) {
      const auto &metrics = stage_metrics_[i];
      if (metrics.call_count == 0)
        continue;

      reports.push_back(
          {stage_names_[i], metrics.getAvgMs(), metrics.getLastMs(),
           metrics.getMinMs(), metrics.getMaxMs(),
           total_time > 0 ? (metrics.total_duration_us * 100.0 / total_time)
                          : 0,
           metrics.call_count});
    }

    return reports;
  }

  // Print formatted report
  void printReport() const {
    auto reports = getReport();
    if (reports.empty())
      return;

    size_t window_num = (total_runs_ - 1) / print_interval_ + 1;
    size_t window_start = (window_num - 1) * print_interval_ + 1;

    LOG_BENCH("========== Pipeline Benchmark (Window ", window_num, ": Runs ",
              window_start, "-", total_runs_, ") ==========");
    LOG_BENCH("Stage                      Avg(ms)  Last(ms)  Min(ms)  Max(ms)  "
              "Usage(%)  Calls");
    LOG_BENCH("---------------------------------------------------------------"
              "------------------");

    double total_avg = 0;
    for (const auto &r : reports) {
      char buffer[120];
      snprintf(buffer, sizeof(buffer),
               "%-25s %7.2f  %8.2f  %7.2f  %7.2f   %6.1f%%  %6zu",
               r.stage_name.c_str(), r.avg_ms, r.last_ms, r.min_ms, r.max_ms,
               r.percentage, r.call_count);
      LOG_BENCH(buffer);
      total_avg += r.avg_ms;
    }

    LOG_BENCH("---------------------------------------------------------------"
              "------------------");
    char total_buffer[120];
    snprintf(total_buffer, sizeof(total_buffer),
             "Total Pipeline:           %7.2f ms", total_avg);
    LOG_BENCH(total_buffer);
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

  // Get metrics for specific stage
  const Metrics *getStageMetrics(const std::string &stage_name) const {
    for (size_t i = 0; i < stage_names_.size(); ++i) {
      if (stage_names_[i] == stage_name) {
        return &stage_metrics_[i];
      }
    }
    return nullptr;
  }

  // Get all stages being tracked
  std::vector<std::string> getTrackedStages() const { return stage_names_; }

  size_t getTotalRuns() const { return total_runs_; }

  // Get access to stages from pipeline
  const std::vector<Stage::Ptr> &getStages() const {
    return pipeline_->getStages();
  }

private:
  void updateMetrics(size_t stage_index, const std::string &stage_name,
                     uint64_t duration_us) {
    // Ensure vectors are large enough
    if (stage_index >= stage_names_.size()) {
      stage_names_.resize(stage_index + 1);
      stage_metrics_.resize(stage_index + 1);
    }

    stage_names_[stage_index] = stage_name;
    auto &metrics = stage_metrics_[stage_index];
    metrics.last_duration_us = duration_us;
    metrics.total_duration_us += duration_us;
    metrics.min_duration_us = std::min(metrics.min_duration_us, duration_us);
    metrics.max_duration_us = std::max(metrics.max_duration_us, duration_us);
    metrics.call_count++;
  }

  Pipeline *pipeline_;
  std::vector<std::string> stage_names_; // Stage names in order
  std::vector<Metrics> stage_metrics_;   // Metrics in order
  size_t total_runs_ = 0;

  // Configuration
  bool auto_print_ = false;
  size_t print_interval_ = 100;
  bool log_each_stage_ = false;
};

} // namespace pipeline

#endif // PIPELINE_CORE_PIPELINE_PROFILER_H