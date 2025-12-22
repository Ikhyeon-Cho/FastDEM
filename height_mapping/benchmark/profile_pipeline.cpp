/*
 * profile_pipeline.cpp
 *
 * Standalone benchmark for HeightMapping pipeline profiling.
 * No ROS dependency - uses mock data and providers.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 */

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <random>

#include "height_mapping/height_mapping.h"
#include "height_mapping/ppl/stage_registry.h"

using namespace height_mapping;
using namespace height_mapping::ppl;

// ==================== Mock Data Generator ====================

std::shared_ptr<PointCloud> generateMockCloud(size_t num_points,
                                               const std::string& frame_id) {
  auto cloud = std::make_shared<PointCloud>();
  cloud->points.reserve(num_points);

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> xy_dist(-5.0f, 5.0f);
  std::uniform_real_distribution<float> z_dist(-0.5f, 2.0f);

  for (size_t i = 0; i < num_points; ++i) {
    cloud->points.emplace_back(xy_dist(gen), xy_dist(gen), z_dist(gen));
  }

  cloud->setFrameId(frame_id);
  cloud->setTimestamp(
      std::chrono::high_resolution_clock::now().time_since_epoch().count());

  return cloud;
}

// ==================== Benchmark with Stage Profiling ====================

void runBenchmarkWithProfiling(MappingPipeline& pipeline,
                               std::shared_ptr<HeightMap> map,
                               size_t num_iterations, size_t points_per_cloud) {
  std::cout << "\n========== Benchmark Configuration ==========" << std::endl;
  std::cout << "Iterations: " << num_iterations << std::endl;
  std::cout << "Points per cloud: " << points_per_cloud << std::endl;
  std::cout << "=============================================\n" << std::endl;

  // Create profiler
  MappingProfiler profiler(pipeline);

  // Identity transforms for benchmark (sensor at robot base, robot at origin)
  Transformf identity = Transformf::Identity();

  // Warm-up
  std::cout << "Warming up..." << std::endl;
  for (int i = 0; i < 10; ++i) {
    auto cloud = generateMockCloud(points_per_cloud, "base_link");
    auto frame = std::make_shared<MappingFrame>(cloud, map);
    frame->pose = identity;
    frame->extrinsic = identity;
    pipeline.run(frame);
  }
  map->clear();
  profiler.reset();

  // Benchmark with profiling
  std::cout << "Running benchmark with stage profiling..." << std::endl;

  std::vector<double> total_durations_ms;
  total_durations_ms.reserve(num_iterations);

  for (size_t i = 0; i < num_iterations; ++i) {
    auto cloud = generateMockCloud(points_per_cloud, "base_link");

    auto frame = std::make_shared<MappingFrame>(cloud, map);
    frame->pose = identity;
    frame->extrinsic = identity;

    auto start = std::chrono::high_resolution_clock::now();
    profiler.run(frame);
    auto end = std::chrono::high_resolution_clock::now();

    double duration_ms =
        std::chrono::duration<double, std::milli>(end - start).count();
    total_durations_ms.push_back(duration_ms);

    if ((i + 1) % 100 == 0) {
      std::cout << "  Completed " << (i + 1) << "/" << num_iterations
                << " iterations" << std::endl;
    }
  }

  // Calculate total statistics
  double sum = 0, min_val = total_durations_ms[0], max_val = total_durations_ms[0];
  for (double d : total_durations_ms) {
    sum += d;
    min_val = std::min(min_val, d);
    max_val = std::max(max_val, d);
  }
  double avg = sum / total_durations_ms.size();

  double sq_sum = 0;
  for (double d : total_durations_ms) {
    sq_sum += (d - avg) * (d - avg);
  }
  double stddev = std::sqrt(sq_sum / total_durations_ms.size());

  // Print total results
  std::cout << "\n========== Total Pipeline Results ==========" << std::endl;
  std::cout << "Iterations: " << num_iterations << std::endl;
  std::cout << "Points per cloud: " << points_per_cloud << std::endl;
  std::cout << "---------------------------------------------" << std::endl;
  std::cout << "Average: " << avg << " ms" << std::endl;
  std::cout << "Min:     " << min_val << " ms" << std::endl;
  std::cout << "Max:     " << max_val << " ms" << std::endl;
  std::cout << "Stddev:  " << stddev << " ms" << std::endl;
  std::cout << "---------------------------------------------" << std::endl;
  std::cout << "Throughput: " << (1000.0 / avg) << " Hz" << std::endl;
  std::cout << "Points/sec: " << (points_per_cloud * 1000.0 / avg) / 1e6
            << " M points/sec" << std::endl;

  // Print stage-by-stage profiling
  std::cout << "\n========== Stage-by-Stage Profiling ==========" << std::endl;
  profiler.printReport();
}

// ==================== Main ====================

int main(int argc, char** argv) {
  // Default parameters
  size_t num_iterations = 500;
  size_t points_per_cloud = 50000;
  std::string config_file;

  // Parse arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-n" && i + 1 < argc) {
      num_iterations = std::stoul(argv[++i]);
    } else if (arg == "-p" && i + 1 < argc) {
      points_per_cloud = std::stoul(argv[++i]);
    } else if (arg == "-c" && i + 1 < argc) {
      config_file = argv[++i];
    } else if (arg == "-h" || arg == "--help") {
      std::cout << "Usage: " << argv[0] << " [options]\n"
                << "Options:\n"
                << "  -n <num>    Number of iterations (default: 500)\n"
                << "  -p <num>    Points per cloud (default: 50000)\n"
                << "  -c <file>   Config file path (required)\n"
                << "  -h          Show this help\n";
      return 0;
    }
  }

  // Config file is now required
  if (config_file.empty()) {
    std::cerr << "Error: Config file is required. Use -c <file> to specify."
              << std::endl;
    std::cerr << "Use -h for help." << std::endl;
    return 1;
  }

  std::cout << "HeightMapping Pipeline Benchmark (with Stage Profiling)"
            << std::endl;
  std::cout << "========================================================\n"
            << std::endl;

  // Register all stages
  registerAllStages();

  // Create HeightMap
  auto map = std::make_shared<height_mapping::HeightMap>();
  map->setGeometry(10.0, 10.0, 0.1);
  map->setFrameId("map");

  // Build pipeline from config file
  try {
    MappingPipeline pipeline;
    pipeline.load(config_file);
    std::cout << "Loaded config from: " << config_file << std::endl;

    std::cout << "Pipeline initialized with " << pipeline.size()
              << " stages\n"
              << std::endl;

    // Run benchmark
    runBenchmarkWithProfiling(pipeline, map, num_iterations, points_per_cloud);

  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
