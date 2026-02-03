// Benchmark: PCD I/O with chunked reading/writing
// Tests memory efficiency and performance on large map files

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/io/pcd_io.hpp"

// Simple memory usage check (Linux only)
size_t getCurrentRSS() {
  std::ifstream stat("/proc/self/status");
  std::string line;
  while (std::getline(stat, line)) {
    if (line.substr(0, 6) == "VmRSS:") {
      size_t kb = 0;
      sscanf(line.c_str(), "VmRSS: %zu", &kb);
      return kb * 1024; // Return bytes
    }
  }
  return 0;
}

std::string formatBytes(size_t bytes) {
  if (bytes >= 1024 * 1024 * 1024)
    return std::to_string(bytes / (1024 * 1024 * 1024)) + " GB";
  if (bytes >= 1024 * 1024)
    return std::to_string(bytes / (1024 * 1024)) + " MB";
  if (bytes >= 1024)
    return std::to_string(bytes / 1024) + " KB";
  return std::to_string(bytes) + " B";
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <pcd_file> [output_file]\n";
    return 1;
  }

  std::string input_file = argv[1];
  std::string output_file = (argc > 2) ? argv[2] : "/tmp/benchmark_out.pcd";

  std::cout << "=== nanoPCL PCD I/O Benchmark ===\n\n";
  std::cout << "Input: " << input_file << "\n\n";

  // Check file size
  std::ifstream fs(input_file, std::ios::binary | std::ios::ate);
  if (!fs.is_open()) {
    std::cerr << "Cannot open file: " << input_file << "\n";
    return 1;
  }
  size_t file_size = fs.tellg();
  fs.close();
  std::cout << "File size: " << formatBytes(file_size) << "\n\n";

  // Measure baseline memory
  size_t mem_before = getCurrentRSS();
  std::cout << "Memory before load: " << formatBytes(mem_before) << "\n";

  // Load benchmark
  nanopcl::PointCloud cloud;
  std::cout << "\n--- Loading ---\n";

  auto t1 = std::chrono::high_resolution_clock::now();
  try {
    cloud = nanopcl::io::loadPCD(input_file);
  } catch (const std::exception& e) {
    std::cerr << "Failed to load PCD file: " << e.what() << "\n";
    return 1;
  }
  auto t2 = std::chrono::high_resolution_clock::now();

  size_t mem_after_load = getCurrentRSS();
  auto load_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

  std::cout << "Points loaded: " << cloud.size() << "\n";
  std::cout << "Has intensity: " << (cloud.hasIntensity() ? "yes" : "no")
            << "\n";
  std::cout << "Load time: " << load_ms << " ms\n";
  std::cout << "Memory after load: " << formatBytes(mem_after_load) << "\n";
  std::cout << "Memory delta: " << formatBytes(mem_after_load - mem_before)
            << "\n";

  // Expected memory: points * (12 bytes xyz + 4 bytes intensity)
  size_t expected_mem = cloud.size() * (sizeof(nanopcl::Point) + sizeof(float));
  std::cout << "Expected cloud size: " << formatBytes(expected_mem) << "\n";
  std::cout << "Overhead ratio: " << std::fixed << std::setprecision(2)
            << static_cast<double>(mem_after_load - mem_before) / expected_mem
            << "x\n";

  // Save benchmark
  std::cout << "\n--- Saving ---\n";
  std::cout << "Output: " << output_file << "\n";

  auto t3 = std::chrono::high_resolution_clock::now();
  try {
    nanopcl::io::savePCD(output_file, cloud);
  } catch (const std::exception& e) {
    std::cerr << "Failed to save PCD file: " << e.what() << "\n";
    return 1;
  }
  auto t4 = std::chrono::high_resolution_clock::now();

  auto save_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();
  std::cout << "Save time: " << save_ms << " ms\n";

  // Check output file size
  std::ifstream out_fs(output_file, std::ios::binary | std::ios::ate);
  size_t out_size = out_fs.tellg();
  out_fs.close();
  std::cout << "Output size: " << formatBytes(out_size) << "\n";

  // Summary
  std::cout << "\n=== Summary ===\n";
  std::cout << "Load: " << load_ms << " ms ("
            << static_cast<double>(file_size) / load_ms / 1000 << " MB/s)\n";
  std::cout << "Save: " << save_ms << " ms ("
            << static_cast<double>(out_size) / save_ms / 1000 << " MB/s)\n";
  std::cout << "Peak memory overhead: "
            << formatBytes(mem_after_load - mem_before - expected_mem) << "\n";

  return 0;
}
