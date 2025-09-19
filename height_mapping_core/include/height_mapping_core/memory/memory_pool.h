/*
 * memory_pool.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_MEMORY_MEMORY_POOL_H
#define HEIGHT_MAPPING_CORE_MEMORY_MEMORY_POOL_H

#include <any>
#include <atomic>
#include <memory>
#include <mutex>
#include <typeinfo>
#include <unordered_map>
#include <vector>

namespace height_mapping::core {

// Simple object pool for frequently allocated objects
template <typename T> class ObjectPool {
public:
  explicit ObjectPool(size_t initial_size = 100) {
    pool_.reserve(initial_size);
    for (size_t i = 0; i < initial_size; ++i) {
      pool_.push_back(std::make_unique<T>());
    }
  }

  std::unique_ptr<T> acquire() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (pool_.empty()) {
      return std::make_unique<T>();
    }

    auto obj = std::move(pool_.back());
    pool_.pop_back();
    return obj;
  }

  void release(std::unique_ptr<T> obj) {
    if (!obj)
      return;

    // Reset object to clean state
    *obj = T{};

    std::lock_guard<std::mutex> lock(mutex_);
    pool_.push_back(std::move(obj));
  }

  size_t available() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pool_.size();
  }

private:
  mutable std::mutex mutex_;
  std::vector<std::unique_ptr<T>> pool_;
};

// Pre-allocated buffer pool for vectors
class VectorPool {
public:
  explicit VectorPool(size_t num_vectors = 100, size_t vector_capacity = 100) {
    for (size_t i = 0; i < num_vectors; ++i) {
      std::vector<float> vec;
      vec.reserve(vector_capacity);
      pool_.push_back(std::move(vec));
    }
  }

  std::vector<float> acquire() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (pool_.empty()) {
      std::vector<float> vec;
      vec.reserve(100); // Default capacity
      return vec;
    }

    auto vec = std::move(pool_.back());
    pool_.pop_back();
    vec.clear(); // Clear contents but keep capacity
    return vec;
  }

  void release(std::vector<float> &&vec) {
    vec.clear(); // Clear contents but keep capacity

    std::lock_guard<std::mutex> lock(mutex_);
    pool_.push_back(std::move(vec));
  }

private:
  std::mutex mutex_;
  std::vector<std::vector<float>> pool_;
};

// Thread-local storage for frequently used temporary containers
class ThreadLocalStorage {
public:
  static ThreadLocalStorage &instance() {
    thread_local ThreadLocalStorage instance;
    return instance;
  }

  // Get a temporary vector (cleared but with reserved capacity)
  std::vector<float> &getTempVector(size_t min_capacity = 0) {
    if (temp_vector_.capacity() < min_capacity) {
      temp_vector_.reserve(min_capacity);
    }
    temp_vector_.clear();
    return temp_vector_;
  }

  // Get a temporary map (cleared but with reserved buckets)
  template <typename K, typename V>
  std::unordered_map<K, V> &getTempMap(size_t min_buckets = 0) {
    auto &map = temp_map_storage_[typeid(std::unordered_map<K, V>).hash_code()];
    if (!map) {
      map = std::make_unique<std::any>(std::unordered_map<K, V>());
    }

    auto &typed_map = *std::any_cast<std::unordered_map<K, V>>(&*map);
    typed_map.clear();
    if (min_buckets > 0 && typed_map.bucket_count() < min_buckets) {
      typed_map.reserve(min_buckets);
    }
    return typed_map;
  }

private:
  ThreadLocalStorage() = default;

  std::vector<float> temp_vector_;
  std::unordered_map<size_t, std::unique_ptr<std::any>> temp_map_storage_;
};

// Pre-allocated ring buffer for streaming data
template <typename T> class RingBuffer {
public:
  explicit RingBuffer(size_t capacity)
      : buffer_(capacity), capacity_(capacity), write_idx_(0), read_idx_(0),
        size_(0) {}

  bool push(const T &item) {
    if (size_ >= capacity_) {
      return false; // Buffer full
    }

    buffer_[write_idx_] = item;
    write_idx_ = (write_idx_ + 1) % capacity_;
    size_++;
    return true;
  }

  bool pop(T &item) {
    if (size_ == 0) {
      return false; // Buffer empty
    }

    item = buffer_[read_idx_];
    read_idx_ = (read_idx_ + 1) % capacity_;
    size_--;
    return true;
  }

  void clear() {
    write_idx_ = 0;
    read_idx_ = 0;
    size_ = 0;
  }

  size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }
  bool full() const { return size_ >= capacity_; }

private:
  std::vector<T> buffer_;
  size_t capacity_;
  size_t write_idx_;
  size_t read_idx_;
  std::atomic<size_t> size_;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_MEMORY_MEMORY_POOL_H