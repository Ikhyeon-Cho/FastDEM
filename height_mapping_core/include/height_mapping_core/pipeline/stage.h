/*
 * stage.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_STAGE_H
#define HEIGHT_MAPPING_CORE_PIPELINE_STAGE_H

#include <chrono>
#include <memory>
#include <string>

namespace height_mapping::core {

// Forward declaration
class ProcessingContext;

// Base interface for all pipeline stages
class IPipelineStage {
public:
  virtual ~IPipelineStage() = default;

  // Core processing method
  virtual void process(ProcessingContext &ctx) = 0;

  // Metadata
  virtual std::string getName() const = 0;
  virtual std::string getType() const { return "Unknown"; }

  // Control flow
  virtual bool isEnabled() const { return enabled_; }
  virtual void setEnabled(bool enabled) { enabled_ = enabled; }

  // Check if this stage can process the given context
  virtual bool canProcess(const ProcessingContext &ctx) const { return true; }

  // Optional: Called once before first process
  virtual void initialize() {}

  // Optional: Called when pipeline is destroyed
  virtual void shutdown() {}

  // Performance metrics
  struct Metrics {
    size_t execution_count = 0;
    double total_time_ms = 0.0;
    double min_time_ms = std::numeric_limits<double>::max();
    double max_time_ms = 0.0;
    double avg_time_ms = 0.0;

    void update(double time_ms) {
      execution_count++;
      total_time_ms += time_ms;
      min_time_ms = std::min(min_time_ms, time_ms);
      max_time_ms = std::max(max_time_ms, time_ms);
      avg_time_ms = total_time_ms / execution_count;
    }

    void reset() {
      execution_count = 0;
      total_time_ms = 0.0;
      min_time_ms = std::numeric_limits<double>::max();
      max_time_ms = 0.0;
      avg_time_ms = 0.0;
    }
  };

  const Metrics &getMetrics() const { return metrics_; }
  void resetMetrics() { metrics_.reset(); }

protected:
  bool enabled_ = true;
  mutable Metrics metrics_;

  // Helper to measure execution time
  template <typename Func> void measureExecution(Func &&func) {
    auto start = std::chrono::high_resolution_clock::now();
    func();
    auto end = std::chrono::high_resolution_clock::now();

    auto duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    metrics_.update(duration.count() / 1000.0);
  }
};

// Convenience base class with common functionality
template <typename Derived> class PipelineStage : public IPipelineStage {
public:
  std::string getName() const override { return Derived::STAGE_NAME; }

  std::string getType() const override { return Derived::STAGE_TYPE; }

  void process(ProcessingContext &ctx) override {
    if (!isEnabled())
      return;
    if (!canProcess(ctx))
      return;

    measureExecution(
        [this, &ctx]() { static_cast<Derived *>(this)->processImpl(ctx); });
  }
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_STAGE_H