/*
 * pipeline.h
 *
 * SEMI-PUBLIC API - Pipeline execution engine.
 *
 * This header defines the Pipeline class that executes stages sequentially.
 * Users typically don't create Pipeline directly - use PipelineBuilder instead.
 * This header is included when you need to store or execute pipelines.
 *
 * Typical usage:
 *   // Create pipeline using builder
 *   auto pipeline = flowpipe::PipelineBuilder::fromFile("config.yaml");
 *
 *   // Execute pipeline
 *   MyContext ctx;
 *   pipeline->process(ctx);
 *
 * Note: Direct Pipeline manipulation (addStage, enableStage) is considered
 * advanced usage. Most users should configure pipelines through
 *PipelineBuilder.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FLOWPIPE_PIPELINE_H
#define FLOWPIPE_PIPELINE_H

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "flowpipe/context.h"
#include "flowpipe/exceptions.h"
#include "flowpipe/stage.h"

namespace flowpipe {

class Pipeline {
 public:
  Pipeline() = default;

  // Disable copy
  Pipeline(const Pipeline &) = delete;
  Pipeline &operator=(const Pipeline &) = delete;

  // Enable move
  Pipeline(Pipeline &&) = default;
  Pipeline &operator=(Pipeline &&) = default;

  // Stage management
  Pipeline &addStage(Stage::Ptr stage) {
    if (stage) {
      stages_.push_back(std::move(stage));
    }
    return *this;
  }

  // Stage access
  const std::vector<Stage::Ptr> &getStages() const { return stages_; }

  Stage *getStage(const std::string &name) {
    auto it = std::find_if(
        stages_.begin(), stages_.end(),
        [&name](const Stage::Ptr &stage) { return stage->getName() == name; });
    return (it != stages_.end()) ? it->get() : nullptr;
  }

  const Stage *getStage(const std::string &name) const {
    auto it = std::find_if(
        stages_.begin(), stages_.end(),
        [&name](const Stage::Ptr &stage) { return stage->getName() == name; });
    return (it != stages_.end()) ? it->get() : nullptr;
  }

  // Processing
  void process(Context &ctx) {
    std::lock_guard<std::mutex> lock(process_mutex_);  // Thread-safe

    if (stages_.empty()) return;

    for (auto &stage : stages_) {
      // Check if stage can process
      if (!stage->isEnabled() || !stage->canProcess(ctx)) {
        continue;
      }

      try {
        stage->process(ctx);

      } catch (const CriticalError &e) {
        // Critical errors always stop the pipeline
        throw;  // Re-throw to caller

      } catch (const RecoverableError &e) {
        // Recoverable errors respect stop_on_error setting
        if (stop_on_error_) {
          throw;  // Re-throw if configured to stop
        }
        // Otherwise continue to next stage

      } catch (const std::exception &e) {
        // Treat unknown exceptions as critical
        throw CriticalError(stage->getName(), e.what());
      }
    }
  }

  // Configuration
  void setStopOnError(bool stop) { stop_on_error_ = stop; }
  bool getStopOnError() const { return stop_on_error_; }

  // Enable/disable stages
  void enableStage(const std::string &name, bool enable) {
    if (auto *stage = getStage(name)) {
      stage->setEnabled(enable);
    }
  }

 private:
  std::vector<Stage::Ptr> stages_;
  mutable std::mutex process_mutex_;
  bool stop_on_error_ = true;
};

}  // namespace flowpipe

#endif  // FLOWPIPE_PIPELINE_H