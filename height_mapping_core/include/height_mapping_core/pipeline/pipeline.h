/*
 * pipeline.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_PIPELINE_H
#define HEIGHT_MAPPING_CORE_PIPELINE_PIPELINE_H

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "height_mapping_core/pipeline/processing_context.h"
#include "height_mapping_core/pipeline/stage.h"

namespace height_mapping::core {

class Pipeline {
public:
  using StagePtr = std::unique_ptr<IPipelineStage>;
  using StageCallback =
      std::function<void(const std::string &, const IPipelineStage::Metrics &)>;

  // Constructor
  Pipeline() = default;

  // Disable copy
  Pipeline(const Pipeline &) = delete;
  Pipeline &operator=(const Pipeline &) = delete;

  // Enable move
  Pipeline(Pipeline &&) = default;
  Pipeline &operator=(Pipeline &&) = default;

  // Stage management
  Pipeline &addStage(StagePtr stage) {
    if (stage) {
      stage->initialize();
      stages_.push_back(std::move(stage));
    }
    return *this;
  }

  template <typename StageType, typename... Args>
  Pipeline &emplaceStage(Args &&... args) {
    return addStage(std::make_unique<StageType>(std::forward<Args>(args)...));
  }

  Pipeline &insertStage(size_t index, StagePtr stage) {
    if (stage && index <= stages_.size()) {
      stage->initialize();
      stages_.insert(stages_.begin() + index, std::move(stage));
    }
    return *this;
  }

  bool removeStage(const std::string &name) {
    auto it = std::find_if(
        stages_.begin(), stages_.end(),
        [&name](const StagePtr &stage) { return stage->getName() == name; });

    if (it != stages_.end()) {
      (*it)->shutdown();
      stages_.erase(it);
      return true;
    }
    return false;
  }

  void clearStages() {
    for (auto &stage : stages_) {
      stage->shutdown();
    }
    stages_.clear();
  }

  // Stage access
  IPipelineStage *getStage(const std::string &name) {
    auto it = std::find_if(
        stages_.begin(), stages_.end(),
        [&name](const StagePtr &stage) { return stage->getName() == name; });
    return (it != stages_.end()) ? it->get() : nullptr;
  }

  const IPipelineStage *getStage(const std::string &name) const {
    auto it = std::find_if(
        stages_.begin(), stages_.end(),
        [&name](const StagePtr &stage) { return stage->getName() == name; });
    return (it != stages_.end()) ? it->get() : nullptr;
  }

  std::vector<std::string> getStageNames() const {
    std::vector<std::string> names;
    names.reserve(stages_.size());
    for (const auto &stage : stages_) {
      names.push_back(stage->getName());
    }
    return names;
  }

  size_t getStageCount() const { return stages_.size(); }
  bool empty() const { return stages_.empty(); }

  // Processing
  void process(ProcessingContext &ctx) {
    // Thread-safe processing with mutex
    std::lock_guard<std::mutex> lock(process_mutex_);

    if (stages_.empty())
      return;

    auto pipeline_start = std::chrono::high_resolution_clock::now();

    // Timing information for each stage
    static int frame_counter = 0;
    bool print_timing = (++frame_counter % 30 == 0); // Print every 30 frames

    // Pre-allocated vector for timing (reused across frames)
    static thread_local std::vector<std::pair<std::string, long>> stage_times;
    stage_times.clear(); // Clear but keep capacity

    for (size_t i = 0; i < stages_.size(); ++i) {
      auto &stage = stages_[i];

      // Check if stage can process
      if (!stage->isEnabled() || !stage->canProcess(ctx)) {
        continue;
      }

      // Process stage with timing
      auto stage_start = std::chrono::high_resolution_clock::now();
      try {
        stage->process(ctx);

        // Call stage callback if set
        if (stage_callback_) {
          stage_callback_(stage->getName(), stage->getMetrics());
        }

        // Check for errors
        if (ctx.hasError() && stop_on_error_) {
          break;
        }

      } catch (const std::exception &e) {
        ctx.setError(stage->getName(), e.what());
        if (stop_on_error_) {
          break;
        }
      }

      auto stage_end = std::chrono::high_resolution_clock::now();
      auto stage_duration =
          std::chrono::duration_cast<std::chrono::microseconds>(stage_end -
                                                                stage_start)
              .count();

      if (print_timing) {
        stage_times.push_back({stage->getName(), stage_duration});
      }
    }

    auto pipeline_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        pipeline_end - pipeline_start);
    ctx.stats().processing_time_ms = duration.count() / 1000.0;

    // Print detailed timing
    if (print_timing && !stage_times.empty()) {
      std::cout << "[PIPELINE TIMING] Total: " << duration.count()
                << " us | Stages: ";
      for (const auto &[name, time] : stage_times) {
        std::cout << name << "=" << time << "us ";
      }
      std::cout << std::endl;
    }
  }

  // Configuration
  void setStopOnError(bool stop) { stop_on_error_ = stop; }
  bool getStopOnError() const { return stop_on_error_; }

  void setStageCallback(StageCallback callback) { stage_callback_ = callback; }

  // Enable/disable stages
  void enableStage(const std::string &name, bool enable) {
    if (auto *stage = getStage(name)) {
      stage->setEnabled(enable);
    }
  }

  void enableAllStages() {
    for (auto &stage : stages_) {
      stage->setEnabled(true);
    }
  }

  void disableAllStages() {
    for (auto &stage : stages_) {
      stage->setEnabled(false);
    }
  }

  // Metrics
  struct PipelineMetrics {
    std::unordered_map<std::string, IPipelineStage::Metrics> stage_metrics;
    double total_pipeline_time_ms = 0.0;
    size_t total_executions = 0;

    void collectFrom(const Pipeline &pipeline) {
      stage_metrics.clear();
      for (const auto &stage : pipeline.stages_) {
        stage_metrics[stage->getName()] = stage->getMetrics();
      }
    }

    std::string toString() const {
      std::stringstream ss;
      ss << "Pipeline Metrics:\n";
      ss << "  Total executions: " << total_executions << "\n";
      ss << "  Total time: " << total_pipeline_time_ms << " ms\n";
      ss << "  Stage Metrics:\n";
      for (const auto &[name, metrics] : stage_metrics) {
        ss << "    " << name << ":\n";
        ss << "      Executions: " << metrics.execution_count << "\n";
        ss << "      Avg time: " << metrics.avg_time_ms << " ms\n";
        ss << "      Min/Max: " << metrics.min_time_ms << "/"
           << metrics.max_time_ms << " ms\n";
      }
      return ss.str();
    }
  };

  PipelineMetrics getMetrics() const {
    PipelineMetrics metrics;
    metrics.collectFrom(*this);
    return metrics;
  }

  void resetMetrics() {
    for (auto &stage : stages_) {
      stage->resetMetrics();
    }
  }

  // Utility
  std::string toString() const {
    std::stringstream ss;
    ss << "Pipeline with " << stages_.size() << " stages:\n";
    for (size_t i = 0; i < stages_.size(); ++i) {
      ss << "  [" << i << "] " << stages_[i]->getName();
      ss << " (" << stages_[i]->getType() << ")";
      if (!stages_[i]->isEnabled()) {
        ss << " [DISABLED]";
      }
      ss << "\n";
    }
    return ss.str();
  }

private:
  std::vector<StagePtr> stages_;
  bool stop_on_error_ = true;
  StageCallback stage_callback_;
  mutable std::mutex process_mutex_; // Protects pipeline processing
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_PIPELINE_H