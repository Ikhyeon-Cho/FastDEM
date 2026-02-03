// nanoPCL Benchmark: PCL Bridge Conversion Overhead
// Measures the cost of converting between PCL types and nanoPCL

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

// nanoPCL
#include <nanopcl/bridge/pcl.hpp>
#include <nanopcl/common.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Timer utility
class Timer {
  std::chrono::high_resolution_clock::time_point start_;

public:
  Timer() { start_ = std::chrono::high_resolution_clock::now(); }

  double elapsed_us() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::micro>(end - start_).count();
  }

  double elapsed_ms() const { return elapsed_us() / 1000.0; }
};

// Prevent compiler optimization
template <typename T>
void doNotOptimize(T& value) {
  asm volatile(""
               : "+m"(value)
               :
               : "memory");
}

void printHeader(const std::string& title) {
  std::cout << "\n"
            << std::string(70, '=') << "\n";
  std::cout << title << "\n";
  std::cout << std::string(70, '=') << "\n";
}

void printSubHeader(const std::string& title) {
  std::cout << "\n--- " << title << " ---\n";
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr createPclCloud(size_t num_points);

template <>
pcl::PointCloud<pcl::PointXYZ>::Ptr createPclCloud<pcl::PointXYZ>(
    size_t num_points) {
  typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->header.frame_id = "base_link";
  cloud->width = num_points;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(num_points);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist(-50.0f, 50.0f);

  for (size_t i = 0; i < num_points; ++i) {
    cloud->points[i].x = dist(gen);
    cloud->points[i].y = dist(gen);
    cloud->points[i].z = dist(gen);
  }

  return cloud;
}

template <>
pcl::PointCloud<pcl::PointXYZI>::Ptr createPclCloud<pcl::PointXYZI>(
    size_t num_points) {
  typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  cloud->header.frame_id = "base_link";
  cloud->width = num_points;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(num_points);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> intensity_dist(0.0f, 1.0f);

  for (size_t i = 0; i < num_points; ++i) {
    cloud->points[i].x = pos_dist(gen);
    cloud->points[i].y = pos_dist(gen);
    cloud->points[i].z = pos_dist(gen);
    cloud->points[i].intensity = intensity_dist(gen);
  }

  return cloud;
}

template <>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr createPclCloud<pcl::PointXYZRGB>(
    size_t num_points) {
  typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud->header.frame_id = "base_link";
  cloud->width = num_points;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(num_points);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);
  std::uniform_int_distribution<uint8_t> color_dist(0, 255);

  for (size_t i = 0; i < num_points; ++i) {
    cloud->points[i].x = pos_dist(gen);
    cloud->points[i].y = pos_dist(gen);
    cloud->points[i].z = pos_dist(gen);
    cloud->points[i].r = color_dist(gen);
    cloud->points[i].g = color_dist(gen);
    cloud->points[i].b = color_dist(gen);
  }

  return cloud;
}

template <>
pcl::PointCloud<pcl::PointXYZINormal>::Ptr createPclCloud<pcl::PointXYZINormal>(
    size_t num_points) {
  typename pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>());
  cloud->header.frame_id = "base_link";
  cloud->width = num_points;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(num_points);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> intensity_dist(0.0f, 1.0f);
  std::uniform_real_distribution<float> normal_dist(-1.0f, 1.0f);

  for (size_t i = 0; i < num_points; ++i) {
    cloud->points[i].x = pos_dist(gen);
    cloud->points[i].y = pos_dist(gen);
    cloud->points[i].z = pos_dist(gen);
    cloud->points[i].intensity = intensity_dist(gen);

    float nx = normal_dist(gen), ny = normal_dist(gen), nz = normal_dist(gen);
    float len = std::sqrt(nx * nx + ny * ny + nz * nz);
    cloud->points[i].normal_x = nx / len;
    cloud->points[i].normal_y = ny / len;
    cloud->points[i].normal_z = nz / len;
  }

  return cloud;
}

nanopcl::PointCloud createNanoPclCloud(size_t num_points, bool with_intensity, bool with_color, bool with_normal) {
  nanopcl::PointCloud cloud(num_points);
  cloud.setFrameId("base_link");

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> intensity_dist(0.0f, 1.0f);
  std::uniform_int_distribution<uint8_t> color_dist(0, 255);
  std::uniform_real_distribution<float> normal_dist(-1.0f, 1.0f);

  if (with_intensity)
    cloud.enableIntensity();
  if (with_color)
    cloud.enableColor();
  if (with_normal)
    cloud.enableNormal();

  for (size_t i = 0; i < num_points; ++i) {
    cloud.add(nanopcl::Point(pos_dist(gen), pos_dist(gen), pos_dist(gen)));

    if (with_intensity)
      cloud.intensity().back() = intensity_dist(gen);
    if (with_color) {
      cloud.color().back() =
          nanopcl::Color{color_dist(gen), color_dist(gen), color_dist(gen)};
    }
    if (with_normal) {
      float nx = normal_dist(gen), ny = normal_dist(gen), nz = normal_dist(gen);
      float len = std::sqrt(nx * nx + ny * ny + nz * nz);
      cloud.normal().back() = Eigen::Vector3f(nx / len, ny / len, nz / len);
    }
  }

  return cloud;
}

struct BenchmarkResult {
  double mean_us;
  double min_us;
  double max_us;
  double throughput_mpts_per_sec;
};

BenchmarkResult runBenchmark(std::function<void()> fn, int iterations, size_t num_points) {
  std::vector<double> times;
  times.reserve(iterations);

  // Warmup
  for (int i = 0; i < 3; ++i)
    fn();

  // Measure
  for (int i = 0; i < iterations; ++i) {
    Timer t;
    fn();
    times.push_back(t.elapsed_us());
  }

  double sum = 0, min_t = times[0], max_t = times[0];
  for (double t : times) {
    sum += t;
    min_t = std::min(min_t, t);
    max_t = std::max(max_t, t);
  }

  BenchmarkResult result;
  result.mean_us = sum / iterations;
  result.min_us = min_t;
  result.max_us = max_t;
  result.throughput_mpts_per_sec = (num_points / result.mean_us); // M pts/sec
  return result;
}

void printResult(const std::string& name, const BenchmarkResult& r) {
  std::cout << std::setw(45) << std::left << name << std::fixed
            << std::setprecision(1) << std::setw(10) << std::right << r.mean_us
            << " us  (min: " << std::setw(8) << r.min_us
            << ", max: " << std::setw(8) << r.max_us << ")  "
            << std::setprecision(2) << r.throughput_mpts_per_sec
            << " M pts/sec\n";
}

int main() {
  const std::vector<size_t> point_counts = {10000, 50000, 100000, 500000};
  const int NUM_ITERATIONS = 50;

  std::cout << "nanoPCL Bridge Benchmark: PCL Conversion Overhead\n";
  std::cout << "Iterations per test: " << NUM_ITERATIONS << "\n";

  // ============================================================================
  // 1. PointXYZ (12 bytes PCL, 12 bytes nanoPCL)
  // ============================================================================
  printHeader("1. PointXYZ (XYZ only)");

  for (size_t num_points : point_counts) {
    printSubHeader(std::to_string(num_points) + " points");

    auto pcl_cloud = createPclCloud<pcl::PointXYZ>(num_points);
    auto nano_cloud = createNanoPclCloud(num_points, false, false, false);

    // PCL → nanoPCL
    nanopcl::PointCloud result_cloud;
    auto r1 = runBenchmark(
        [&]() {
          result_cloud = nanopcl::from(*pcl_cloud);
          doNotOptimize(result_cloud[0].x());
        },
        NUM_ITERATIONS,
        num_points);
    printResult("PCL PointXYZ -> nanoPCL", r1);

    // nanoPCL → PCL
    pcl::PointCloud<pcl::PointXYZ> result_pcl;
    auto r2 = runBenchmark(
        [&]() {
          result_pcl = nanopcl::to<pcl::PointCloud<pcl::PointXYZ>>(nano_cloud);
          doNotOptimize(result_pcl.points[0].x);
        },
        NUM_ITERATIONS,
        num_points);
    printResult("nanoPCL -> PCL PointXYZ", r2);
  }

  // ============================================================================
  // 2. PointXYZI (32 bytes PCL, 16 bytes nanoPCL)
  // ============================================================================
  printHeader("2. PointXYZI (XYZ + Intensity)");

  for (size_t num_points : point_counts) {
    printSubHeader(std::to_string(num_points) + " points");

    auto pcl_cloud = createPclCloud<pcl::PointXYZI>(num_points);
    auto nano_cloud = createNanoPclCloud(num_points, true, false, false);

    nanopcl::PointCloud result_cloud;
    auto r1 = runBenchmark(
        [&]() {
          result_cloud = nanopcl::from(*pcl_cloud);
          doNotOptimize(result_cloud[0].x());
        },
        NUM_ITERATIONS,
        num_points);
    printResult("PCL PointXYZI -> nanoPCL", r1);

    pcl::PointCloud<pcl::PointXYZI> result_pcl;
    auto r2 = runBenchmark(
        [&]() {
          result_pcl = nanopcl::to<pcl::PointCloud<pcl::PointXYZI>>(nano_cloud);
          doNotOptimize(result_pcl.points[0].x);
        },
        NUM_ITERATIONS,
        num_points);
    printResult("nanoPCL -> PCL PointXYZI", r2);
  }

  // ============================================================================
  // 3. PointXYZRGB (32 bytes PCL, 15 bytes nanoPCL)
  // ============================================================================
  printHeader("3. PointXYZRGB (XYZ + Color)");

  for (size_t num_points : point_counts) {
    printSubHeader(std::to_string(num_points) + " points");

    auto pcl_cloud = createPclCloud<pcl::PointXYZRGB>(num_points);
    auto nano_cloud = createNanoPclCloud(num_points, false, true, false);

    nanopcl::PointCloud result_cloud;
    auto r1 = runBenchmark(
        [&]() {
          result_cloud = nanopcl::from(*pcl_cloud);
          doNotOptimize(result_cloud[0].x());
        },
        NUM_ITERATIONS,
        num_points);
    printResult("PCL PointXYZRGB -> nanoPCL", r1);

    pcl::PointCloud<pcl::PointXYZRGB> result_pcl;
    auto r2 = runBenchmark(
        [&]() {
          result_pcl =
              nanopcl::to<pcl::PointCloud<pcl::PointXYZRGB>>(nano_cloud);
          doNotOptimize(result_pcl.points[0].x);
        },
        NUM_ITERATIONS,
        num_points);
    printResult("nanoPCL -> PCL PointXYZRGB", r2);
  }

  // ============================================================================
  // 4. PointXYZINormal (48 bytes PCL, 28 bytes nanoPCL)
  // ============================================================================
  printHeader("4. PointXYZINormal (XYZ + Intensity + Normal)");

  for (size_t num_points : point_counts) {
    printSubHeader(std::to_string(num_points) + " points");

    auto pcl_cloud = createPclCloud<pcl::PointXYZINormal>(num_points);
    auto nano_cloud = createNanoPclCloud(num_points, true, false, true);

    nanopcl::PointCloud result_cloud;
    auto r1 = runBenchmark(
        [&]() {
          result_cloud = nanopcl::from(*pcl_cloud);
          doNotOptimize(result_cloud[0].x());
        },
        NUM_ITERATIONS,
        num_points);
    printResult("PCL PointXYZINormal -> nanoPCL", r1);

    pcl::PointCloud<pcl::PointXYZINormal> result_pcl;
    auto r2 = runBenchmark(
        [&]() {
          result_pcl =
              nanopcl::to<pcl::PointCloud<pcl::PointXYZINormal>>(nano_cloud);
          doNotOptimize(result_pcl.points[0].x);
        },
        NUM_ITERATIONS,
        num_points);
    printResult("nanoPCL -> PCL PointXYZINormal", r2);
  }

  // ============================================================================
  // 5. Shared Pointer Variants
  // ============================================================================
  printHeader("5. SMART POINTER CONVENIENCE API");

  const size_t ptr_test_points = 100000;
  printSubHeader(std::to_string(ptr_test_points) + " points");

  auto pcl_ptr = createPclCloud<pcl::PointXYZI>(ptr_test_points);
  auto nano_cloud = createNanoPclCloud(ptr_test_points, true, false, false);

  // PCL::Ptr → nanoPCL
  nanopcl::PointCloud result;
  auto r1 = runBenchmark(
      [&]() {
        result = nanopcl::from<pcl::PointXYZI>(pcl_ptr);
        doNotOptimize(result[0].x());
      },
      NUM_ITERATIONS,
      ptr_test_points);
  printResult("PCL::Ptr -> nanoPCL (from<T>(ptr))", r1);

  // nanoPCL → PCL::Ptr
  pcl::PointCloud<pcl::PointXYZI>::Ptr result_ptr;
  auto r2 = runBenchmark(
      [&]() {
        result_ptr = nanopcl::toPclPtr<pcl::PointXYZI>(nano_cloud);
        doNotOptimize(result_ptr->points[0].x);
      },
      NUM_ITERATIONS,
      ptr_test_points);
  printResult("nanoPCL -> PCL::Ptr (toPclPtr<T>())", r2);

  // ============================================================================
  // 6. Memory Bandwidth Analysis
  // ============================================================================
  printHeader("6. MEMORY BANDWIDTH ANALYSIS (500k points)");

  const size_t bandwidth_points = 500000;

  printSubHeader("PointXYZI conversion bandwidth");

  auto pcl_xyzi = createPclCloud<pcl::PointXYZI>(bandwidth_points);
  auto nano_xyzi = createNanoPclCloud(bandwidth_points, true, false, false);

  // Calculate theoretical data sizes
  size_t pcl_size = bandwidth_points * sizeof(pcl::PointXYZI);
  size_t nano_size = bandwidth_points * (sizeof(nanopcl::Point) + sizeof(float));

  std::cout << "PCL PointXYZI size: " << pcl_size / (1024.0 * 1024.0)
            << " MB\n";
  std::cout << "nanoPCL (xyz+i) size: " << nano_size / (1024.0 * 1024.0)
            << " MB\n";

  // Measure PCL → nanoPCL
  {
    Timer t;
    auto cloud = nanopcl::from(*pcl_xyzi);
    doNotOptimize(cloud[0].x());
    double elapsed_us = t.elapsed_us();
    double bandwidth_gbps =
        (pcl_size + nano_size) / (elapsed_us * 1e-6) / (1024.0 * 1024.0 * 1024.0);
    std::cout << "PCL -> nanoPCL: " << std::fixed << std::setprecision(2)
              << elapsed_us / 1000.0 << " ms, " << bandwidth_gbps
              << " GB/s effective bandwidth\n";
  }

  // Measure nanoPCL → PCL
  {
    Timer t;
    auto cloud = nanopcl::to<pcl::PointCloud<pcl::PointXYZI>>(nano_xyzi);
    doNotOptimize(cloud.points[0].x);
    double elapsed_us = t.elapsed_us();
    double bandwidth_gbps =
        (pcl_size + nano_size) / (elapsed_us * 1e-6) / (1024.0 * 1024.0 * 1024.0);
    std::cout << "nanoPCL -> PCL: " << std::fixed << std::setprecision(2)
              << elapsed_us / 1000.0 << " ms, " << bandwidth_gbps
              << " GB/s effective bandwidth\n";
  }

  // ============================================================================
  // SUMMARY
  // ============================================================================
  printHeader("SUMMARY");
  std::cout << R"(
PCL <-> nanoPCL Conversion Analysis:

Memory Layout Comparison:
  Type              | PCL (AoS) | nanoPCL (SoA) | Savings
  ------------------|-----------|---------------|--------
  PointXYZ          | 16 bytes  | 12 bytes      | 25%
  PointXYZI         | 32 bytes  | 16 bytes      | 50%
  PointXYZRGB       | 32 bytes  | 15 bytes      | 53%
  PointXYZINormal   | 48 bytes  | 28 bytes      | 42%

Conversion is memory-bound:
  - Throughput ~X M points/sec (depends on point type)
  - Cost proportional to data size
  - nanoPCL's smaller footprint = faster conversion

Recommendations:
  - Convert once at boundaries, process in nanoPCL format
  - Use toPclPtr<T>() when PCL algorithms are needed
  - Consider keeping hot paths in native format
)";

  return 0;
}
