/*
 * processing_context_pool.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_PROCESSING_CONTEXT_POOL_H
#define HEIGHT_MAPPING_CORE_PIPELINE_PROCESSING_CONTEXT_POOL_H

#include "height_mapping_core/memory/memory_pool.h"
#include "height_mapping_core/pipeline/processing_context.h"
#include <mutex>
#include <queue>

namespace height_mapping::core {

// Pool for ProcessingContext objects to avoid repeated allocation
class ProcessingContextPool {
public:
  static ProcessingContextPool &instance() {
    static ProcessingContextPool pool;
    return pool;
  }

  // Acquire a frame from the pool
  std::unique_ptr<ProcessingContext> acquire() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (pool_.empty()) {
      auto frame = std::make_unique<ProcessingContext>();
      configureFrame(*frame);
      return frame;
    }

    auto frame = std::move(pool_.front());
    pool_.pop();
    resetFrame(*frame);
    return frame;
  }

  // Return a frame to the pool
  void release(std::unique_ptr<ProcessingContext> frame) {
    if (!frame)
      return;

    // Clear frame data but keep allocated capacity
    resetFrame(*frame);

    std::lock_guard<std::mutex> lock(mutex_);
    if (pool_.size() < max_pool_size_) {
      pool_.push(std::move(frame));
    }
    // If pool is full, let the frame be destroyed
  }

  // Pre-allocate frames
  void preallocate(size_t count) {
    std::lock_guard<std::mutex> lock(mutex_);

    for (size_t i = 0; i < count && pool_.size() < max_pool_size_; ++i) {
      auto frame = std::make_unique<ProcessingContext>();
      configureFrame(*frame);
      pool_.push(std::move(frame));
    }
  }

  size_t available() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pool_.size();
  }

private:
  ProcessingContextPool() = default;

  void configureFrame(ProcessingContext &ctx) {
    // Pre-allocate cloud capacity
    if (ctx.cloud().points.capacity() < 100000) {
      ctx.cloud().points.reserve(100000); // Typical cloud size
    }
  }

  void resetFrame(ProcessingContext &ctx) {
    // Clear cloud but keep capacity
    ctx.cloud().points.clear();
    ctx.cloud().frame_id.clear();
    ctx.cloud().timestamp = 0;

    // Clear metadata but keep map structure
    ctx.clearMetadata();

    // Clear errors
    ctx.clearErrors();

    // Reset stats
    ctx.stats() = ProcessingContext::Statistics{};
  }

  mutable std::mutex mutex_;
  std::queue<std::unique_ptr<ProcessingContext>> pool_;
  static constexpr size_t max_pool_size_ = 10;
};

// RAII wrapper for automatic frame return to pool
class PooledProcessingContext {
public:
  PooledProcessingContext() : frame_(ProcessingContextPool::instance().acquire()) {}

  ~PooledProcessingContext() {
    if (frame_) {
      ProcessingContextPool::instance().release(std::move(frame_));
    }
  }

  // Move-only semantics
  PooledProcessingContext(PooledProcessingContext &&other) noexcept
      : frame_(std::move(other.frame_)) {}

  PooledProcessingContext &operator=(PooledProcessingContext &&other) noexcept {
    if (this != &other) {
      if (frame_) {
        ProcessingContextPool::instance().release(std::move(frame_));
      }
      frame_ = std::move(other.frame_);
    }
    return *this;
  }

  // Delete copy
  PooledProcessingContext(const PooledProcessingContext &) = delete;
  PooledProcessingContext &operator=(const PooledProcessingContext &) = delete;

  // Access operators
  ProcessingContext *operator->() { return frame_.get(); }
  const ProcessingContext *operator->() const { return frame_.get(); }
  ProcessingContext &operator*() { return *frame_; }
  const ProcessingContext &operator*() const { return *frame_; }

  ProcessingContext *get() { return frame_.get(); }
  const ProcessingContext *get() const { return frame_.get(); }

private:
  std::unique_ptr<ProcessingContext> frame_;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_PROCESSING_CONTEXT_POOL_H