// nanoPCL Benchmark Framework
// Copyright (c) 2025 Ikhyeon Cho <tre0430@korea.ac.kr>
// SPDX-License-Identifier: MIT
//
// Standardized benchmarking utilities for consistent, reproducible measurements.
//
// Features:
//   - Unified iteration policy (FAST/MEDIUM/SLOW)
//   - Statistical analysis with 95% confidence intervals
//   - Outlier removal (IQR method)
//   - Platform information capture
//   - Consistent output formatting
//
// Usage:
//   #include "benchmark_common.hpp"
//
//   int main() {
//     benchmark::printHeader("VoxelGrid Benchmark");
//     benchmark::PlatformInfo::capture().print();
//
//     auto stats = benchmark::run([&]() {
//       filters::voxelGrid(cloud, 0.1f);
//     });
//
//     benchmark::printResult("VoxelGrid", stats);
//     return 0;
//   }

#ifndef NANOPCL_BENCHMARK_COMMON_HPP
#define NANOPCL_BENCHMARK_COMMON_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace benchmark {

// =============================================================================
// Iteration Policy
// =============================================================================

/// Standardized iteration counts based on operation cost
struct IterationPolicy {
  static constexpr int FAST = 100;    ///< Operations < 1ms
  static constexpr int MEDIUM = 50;   ///< Operations 1-100ms
  static constexpr int SLOW = 20;     ///< Operations > 100ms
  static constexpr int WARMUP = 5;    ///< Always 5 warmup iterations

  /// Auto-select based on single-run time (ms)
  static int autoSelect(double single_run_ms) {
    if (single_run_ms < 1.0) return FAST;
    if (single_run_ms < 100.0) return MEDIUM;
    return SLOW;
  }
};

// =============================================================================
// Statistics
// =============================================================================

/// Comprehensive statistics with confidence intervals
struct Stats {
  double mean = 0.0;
  double stddev = 0.0;
  double min = 0.0;
  double max = 0.0;
  double median = 0.0;
  double ci_95_low = 0.0;   ///< 95% confidence interval lower bound
  double ci_95_high = 0.0;  ///< 95% confidence interval upper bound
  size_t n = 0;             ///< Sample count after outlier removal
  size_t n_outliers = 0;    ///< Number of outliers removed

  /// Confidence interval half-width
  double ci_95() const { return (ci_95_high - ci_95_low) / 2.0; }

  /// Compute statistics from samples with optional outlier removal
  static Stats compute(std::vector<double> samples, bool remove_outliers = true) {
    Stats s;
    if (samples.empty()) return s;

    // Sort for median and IQR calculation
    std::sort(samples.begin(), samples.end());

    // Remove outliers using IQR method (only if enough samples)
    if (remove_outliers && samples.size() >= 10) {
      size_t q1_idx = samples.size() / 4;
      size_t q3_idx = 3 * samples.size() / 4;
      double q1 = samples[q1_idx];
      double q3 = samples[q3_idx];
      double iqr = q3 - q1;
      double lower = q1 - 1.5 * iqr;
      double upper = q3 + 1.5 * iqr;

      size_t original_size = samples.size();
      samples.erase(
          std::remove_if(samples.begin(), samples.end(),
                         [&](double x) { return x < lower || x > upper; }),
          samples.end());
      s.n_outliers = original_size - samples.size();

      // Re-sort if outliers removed
      if (s.n_outliers > 0) {
        std::sort(samples.begin(), samples.end());
      }
    }

    if (samples.empty()) return s;

    s.n = samples.size();
    s.min = samples.front();
    s.max = samples.back();
    s.median = samples[samples.size() / 2];

    // Mean
    double sum = std::accumulate(samples.begin(), samples.end(), 0.0);
    s.mean = sum / static_cast<double>(s.n);

    // Standard deviation
    double sq_sum = 0.0;
    for (double v : samples) {
      double diff = v - s.mean;
      sq_sum += diff * diff;
    }
    s.stddev = std::sqrt(sq_sum / static_cast<double>(s.n));

    // 95% confidence interval (t-distribution approximation for n > 30)
    // For smaller n, using t-value ~2.0 is reasonable approximation
    double t_value = (s.n >= 30) ? 1.96 : 2.0;
    double sem = s.stddev / std::sqrt(static_cast<double>(s.n));
    s.ci_95_low = s.mean - t_value * sem;
    s.ci_95_high = s.mean + t_value * sem;

    return s;
  }
};

// =============================================================================
// Platform Information
// =============================================================================

/// Captures and reports system/compiler information for reproducibility
struct PlatformInfo {
  std::string cpu_model;
  std::string compiler;
  std::string optimization;
  bool openmp_enabled = false;
  int max_threads = 1;
  bool simd_avx = false;
  bool simd_avx2 = false;

  /// Capture current platform information
  static PlatformInfo capture() {
    PlatformInfo info;

    // CPU model (Linux-specific)
    std::ifstream cpuinfo("/proc/cpuinfo");
    if (cpuinfo.is_open()) {
      std::string line;
      while (std::getline(cpuinfo, line)) {
        if (line.find("model name") != std::string::npos) {
          size_t pos = line.find(':');
          if (pos != std::string::npos) {
            info.cpu_model = line.substr(pos + 2);
          }
          break;
        }
      }
    }
    if (info.cpu_model.empty()) {
      info.cpu_model = "Unknown CPU";
    }

    // Compiler
#if defined(__clang__)
    info.compiler = "clang " + std::to_string(__clang_major__) + "." +
                    std::to_string(__clang_minor__);
#elif defined(__GNUC__)
    info.compiler = "g++ " + std::to_string(__GNUC__) + "." +
                    std::to_string(__GNUC_MINOR__);
#elif defined(_MSC_VER)
    info.compiler = "MSVC " + std::to_string(_MSC_VER);
#else
    info.compiler = "Unknown";
#endif

    // Optimization level (check common compiler flags)
#if defined(NDEBUG) || defined(__OPTIMIZE__)
    info.optimization = "Release";
#else
    info.optimization = "Debug";
#endif

    // OpenMP
#ifdef _OPENMP
    info.openmp_enabled = true;
    info.max_threads = omp_get_max_threads();
#endif

    // SIMD (compile-time detection)
#ifdef __AVX2__
    info.simd_avx2 = true;
    info.simd_avx = true;
#elif defined(__AVX__)
    info.simd_avx = true;
#endif

    return info;
  }

  /// Print platform information
  void print(std::ostream& os = std::cout) const {
    os << "Platform: " << cpu_model << "\n";
    os << "Compiler: " << compiler << " (" << optimization << ")\n";
    os << "OpenMP:   " << (openmp_enabled ? "Enabled" : "Disabled");
    if (openmp_enabled) {
      os << " (max " << max_threads << " threads)";
    }
    os << "\n";
    os << "SIMD:     ";
    if (simd_avx2)
      os << "AVX2";
    else if (simd_avx)
      os << "AVX";
    else
      os << "SSE";
    os << "\n";
  }
};

// =============================================================================
// Timer Utilities
// =============================================================================

using Clock = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double, std::milli>;

/// Prevent compiler from optimizing away benchmark code
template <typename T>
inline void doNotOptimize(T const& value) {
#if defined(__clang__)
  asm volatile("" : : "g"(value) : "memory");
#elif defined(__GNUC__)
  asm volatile("" : : "g"(value) : "memory");
#else
  // MSVC: use volatile read
  static volatile T sink;
  sink = value;
#endif
}

/// Force memory synchronization
inline void clobberMemory() {
#if defined(__clang__) || defined(__GNUC__)
  asm volatile("" : : : "memory");
#else
  std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
}

// =============================================================================
// Benchmark Runner
// =============================================================================

/// Run a benchmark with warmup and statistical analysis
/// @param func Function to benchmark (should return void or value to prevent optimization)
/// @param iterations Number of timed iterations
/// @param warmup Number of warmup iterations
/// @param remove_outliers Whether to remove outliers from statistics
/// @return Statistics of the measured times in milliseconds
template <typename Func>
Stats run(Func func, int iterations = IterationPolicy::MEDIUM,
          int warmup = IterationPolicy::WARMUP, bool remove_outliers = true) {
  std::vector<double> times;
  times.reserve(iterations);

  // Warmup phase (not measured)
  for (int i = 0; i < warmup; ++i) {
    auto result = func();
    doNotOptimize(result);
    clobberMemory();
  }

  // Measurement phase
  for (int i = 0; i < iterations; ++i) {
    auto start = Clock::now();
    auto result = func();
    auto end = Clock::now();

    doNotOptimize(result);
    clobberMemory();

    Duration elapsed = end - start;
    times.push_back(elapsed.count());
  }

  return Stats::compute(times, remove_outliers);
}

/// Run benchmark for void functions
template <typename Func>
Stats runVoid(Func func, int iterations = IterationPolicy::MEDIUM,
              int warmup = IterationPolicy::WARMUP,
              bool remove_outliers = true) {
  std::vector<double> times;
  times.reserve(iterations);

  // Warmup
  for (int i = 0; i < warmup; ++i) {
    func();
    clobberMemory();
  }

  // Measurement
  for (int i = 0; i < iterations; ++i) {
    auto start = Clock::now();
    func();
    auto end = Clock::now();

    clobberMemory();

    Duration elapsed = end - start;
    times.push_back(elapsed.count());
  }

  return Stats::compute(times, remove_outliers);
}

/// Auto-calibrating benchmark runner
/// Runs once to determine iteration count, then runs full benchmark
template <typename Func>
Stats runAuto(Func func, int warmup = IterationPolicy::WARMUP,
              bool remove_outliers = true) {
  // Single calibration run
  auto start = Clock::now();
  auto result = func();
  auto end = Clock::now();
  doNotOptimize(result);

  Duration elapsed = end - start;
  int iterations = IterationPolicy::autoSelect(elapsed.count());

  return run(func, iterations, warmup, remove_outliers);
}

// =============================================================================
// Output Formatting
// =============================================================================

/// Print benchmark header with title
inline void printHeader(const std::string& title, std::ostream& os = std::cout) {
  os << "\n";
  os << std::string(70, '=') << "\n";
  os << " " << title << "\n";
  os << std::string(70, '=') << "\n";
}

/// Print section separator
inline void printSection(const std::string& title,
                         std::ostream& os = std::cout) {
  os << "\n--- " << title << " ---\n\n";
}

/// Print benchmark result with confidence interval
inline void printResult(const std::string& name, const Stats& stats,
                        const std::string& unit = "ms",
                        std::ostream& os = std::cout) {
  os << std::left << std::setw(32) << name << std::right << std::fixed
     << std::setprecision(3) << std::setw(10) << stats.mean << " " << unit
     << " ± " << std::setw(6) << std::setprecision(3) << stats.ci_95() << " (n="
     << stats.n;
  if (stats.n_outliers > 0) {
    os << ", " << stats.n_outliers << " outliers";
  }
  os << ")\n";
}

/// Print result with speedup relative to baseline
inline void printResultWithSpeedup(const std::string& name, const Stats& stats,
                                   double baseline_mean,
                                   const std::string& unit = "ms",
                                   std::ostream& os = std::cout) {
  double speedup = baseline_mean / stats.mean;
  os << std::left << std::setw(32) << name << std::right << std::fixed
     << std::setprecision(3) << std::setw(10) << stats.mean << " " << unit
     << " ± " << std::setw(6) << stats.ci_95() << "  " << std::setw(6)
     << std::setprecision(2) << speedup << "x\n";
}

/// Print table header for comparison benchmarks
inline void printTableHeader(const std::vector<std::string>& columns,
                             std::ostream& os = std::cout) {
  for (const auto& col : columns) {
    os << std::setw(14) << col;
  }
  os << "\n" << std::string(14 * columns.size(), '-') << "\n";
}

/// Print throughput result (points per second)
inline void printThroughput(const std::string& name, const Stats& stats,
                            size_t num_points, std::ostream& os = std::cout) {
  double throughput_mpts = num_points / stats.mean / 1000.0;  // Million pts/sec
  os << std::left << std::setw(32) << name << std::right << std::fixed
     << std::setprecision(3) << std::setw(10) << stats.mean << " ms"
     << "  " << std::setw(8) << std::setprecision(2) << throughput_mpts
     << " Mpts/s\n";
}

/// Print benchmark footer with summary
inline void printFooter(const std::string& conclusion = "",
                        std::ostream& os = std::cout) {
  os << "\n" << std::string(70, '-') << "\n";
  if (!conclusion.empty()) {
    os << "Conclusion: " << conclusion << "\n";
  }
}

// =============================================================================
// Cache Control (for cold-cache benchmarks)
// =============================================================================

/// Flush CPU caches by touching large buffer
/// @param size_mb Size of flush buffer in megabytes (default: 64MB > L3 cache)
inline void flushCache(size_t size_mb = 64) {
  static std::vector<char> buffer;
  size_t size_bytes = size_mb * 1024 * 1024;

  if (buffer.size() != size_bytes) {
    buffer.resize(size_bytes);
  }

  // Touch every cache line (64 bytes)
  volatile char sink = 0;
  for (size_t i = 0; i < size_bytes; i += 64) {
    buffer[i] = static_cast<char>(i);
    sink += buffer[i];
  }
  doNotOptimize(sink);
}

// =============================================================================
// Data Generation Utilities
// =============================================================================

/// Seeded random number generator for reproducible benchmarks
inline std::mt19937& getRng(uint32_t seed = 42) {
  static std::mt19937 rng(seed);
  return rng;
}

/// Reset RNG to specific seed
inline void resetRng(uint32_t seed = 42) { getRng(seed).seed(seed); }

}  // namespace benchmark

#endif  // NANOPCL_BENCHMARK_COMMON_HPP
