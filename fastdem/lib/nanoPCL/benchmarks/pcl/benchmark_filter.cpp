// nanoPCL vs PCL Benchmark: Filter Performance
// Compares filter operations between nanoPCL and PCL

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

// nanoPCL
#include <nanopcl/common.hpp>

// PCL
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class Timer {
  std::chrono::high_resolution_clock::time_point start_;

public:
  Timer()
      : start_(std::chrono::high_resolution_clock::now()) {}
  double elapsed_ms() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }
};

template <typename T>
void doNotOptimize(T& value) {
  asm volatile(""
               : "+r"(value)
               :
               : "memory");
}

void printHeader(const std::string& title) {
  std::cout << "\n"
            << std::string(70, '=') << "\n";
  std::cout << title << "\n";
  std::cout << std::string(70, '=') << "\n";
}

void printResult(const std::string& name, double time_ms, size_t in_pts, size_t out_pts, double baseline_ms = 0) {
  double throughput = in_pts / time_ms / 1000.0;
  std::cout << std::left << std::setw(30) << name << std::right << std::fixed
            << std::setprecision(3) << std::setw(10) << time_ms << " ms"
            << std::setprecision(2) << std::setw(12) << throughput << " Mpts/s"
            << std::setw(10) << out_pts;
  if (baseline_ms > 0) {
    std::cout << std::setprecision(2) << std::setw(8) << (baseline_ms / time_ms)
              << "x";
  }
  std::cout << "\n";
}

int main() {
  std::cout << "nanoPCL vs PCL Filter Benchmark\n";
  std::cout << std::string(70, '=') << "\n";

  const int ITERATIONS = 20;
  std::vector<size_t> point_counts = {100000, 500000, 1000000};

  // Random generator
  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos(-50.0f, 50.0f);
  std::uniform_real_distribution<float> val(0.0f, 1.0f);

  // =========================================================================
  // VOXEL GRID BENCHMARK
  // =========================================================================
  printHeader("1. VOXEL GRID FILTER (voxel_size = 0.5m)");

  for (size_t NUM_POINTS : point_counts) {
    std::cout << "\n--- " << NUM_POINTS << " points ---\n";
    std::cout << std::left << std::setw(30) << "Method" << std::right
              << std::setw(14) << "Time" << std::setw(14) << "Throughput"
              << std::setw(10) << "Output" << std::setw(8) << "Speedup"
              << "\n";
    std::cout << std::string(70, '-') << "\n";

    const float VOXEL_SIZE = 0.5f;
    double pcl_time = 0;
    size_t pcl_out = 0, nano_out = 0;

    // PCL VoxelGrid
    {
      double total = 0;
      for (int iter = 0; iter < ITERATIONS; ++iter) {
        gen.seed(42 + iter);

        // Create input cloud (using make_shared for safer memory management)
        auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud->width = NUM_POINTS;
        cloud->height = 1;
        cloud->is_dense = true;
        cloud->points.resize(NUM_POINTS);
        for (size_t i = 0; i < NUM_POINTS; ++i) {
          cloud->points[i].x = pos(gen);
          cloud->points[i].y = pos(gen);
          cloud->points[i].z = pos(gen);
          cloud->points[i].intensity = val(gen);
        }

        auto filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(cloud);
        vg.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);

        Timer t;
        vg.filter(*filtered);
        total += t.elapsed_ms();
        pcl_out = filtered->size();
        doNotOptimize(pcl_out);
      }
      pcl_time = total / ITERATIONS;
      printResult("[PCL] VoxelGrid", pcl_time, NUM_POINTS, pcl_out);
    }

    // nanoPCL voxelGrid (copy)
    {
      double total = 0;
      for (int iter = 0; iter < ITERATIONS; ++iter) {
        gen.seed(42 + iter);

        nanopcl::PointCloud cloud("benchmark");
        cloud.reserve(NUM_POINTS);
        cloud.enableIntensity();
        for (size_t i = 0; i < NUM_POINTS; ++i) {
          cloud.add(nanopcl::Point(pos(gen), pos(gen), pos(gen)),
                    nanopcl::Intensity(val(gen)));
        }

        Timer t;
        auto filtered = nanopcl::filters::voxelGrid(cloud, VOXEL_SIZE);
        total += t.elapsed_ms();
        nano_out = filtered.size();
        doNotOptimize(nano_out);
      }
      double nano_time = total / ITERATIONS;
      printResult("[nanoPCL] voxelGrid (copy)", nano_time, NUM_POINTS, nano_out, pcl_time);
    }

    // nanoPCL voxelGrid (move)
    {
      double total = 0;
      for (int iter = 0; iter < ITERATIONS; ++iter) {
        gen.seed(42 + iter);

        nanopcl::PointCloud cloud("benchmark");
        cloud.reserve(NUM_POINTS);
        cloud.enableIntensity();
        for (size_t i = 0; i < NUM_POINTS; ++i) {
          cloud.add(nanopcl::Point(pos(gen), pos(gen), pos(gen)),
                    nanopcl::Intensity(val(gen)));
        }

        Timer t;
        auto filtered =
            nanopcl::filters::voxelGrid(std::move(cloud), VOXEL_SIZE);
        total += t.elapsed_ms();
        nano_out = filtered.size();
        doNotOptimize(nano_out);
      }
      double nano_time = total / ITERATIONS;
      printResult("[nanoPCL] voxelGrid (move)", nano_time, NUM_POINTS, nano_out, pcl_time);
    }
  }

  // =========================================================================
  // PASSTHROUGH / CROPZ BENCHMARK
  // =========================================================================
  printHeader("2. PASSTHROUGH / CROP Z FILTER (z: -10 to 10)");

  for (size_t NUM_POINTS : point_counts) {
    std::cout << "\n--- " << NUM_POINTS << " points ---\n";
    std::cout << std::left << std::setw(30) << "Method" << std::right
              << std::setw(14) << "Time" << std::setw(14) << "Throughput"
              << std::setw(10) << "Output" << std::setw(8) << "Speedup"
              << "\n";
    std::cout << std::string(70, '-') << "\n";

    const float Z_MIN = -10.0f, Z_MAX = 10.0f;
    double pcl_time = 0;
    size_t pcl_out = 0, nano_out = 0;

    // PCL PassThrough
    {
      double total = 0;
      for (int iter = 0; iter < ITERATIONS; ++iter) {
        gen.seed(42 + iter);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>);
        cloud->width = NUM_POINTS;
        cloud->height = 1;
        cloud->is_dense = true;
        cloud->points.resize(NUM_POINTS);
        for (size_t i = 0; i < NUM_POINTS; ++i) {
          cloud->points[i].x = pos(gen);
          cloud->points[i].y = pos(gen);
          cloud->points[i].z = pos(gen);
          cloud->points[i].intensity = val(gen);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
            new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PassThrough<pcl::PointXYZI> pt;
        pt.setInputCloud(cloud);
        pt.setFilterFieldName("z");
        pt.setFilterLimits(Z_MIN, Z_MAX);

        Timer t;
        pt.filter(*filtered);
        total += t.elapsed_ms();
        pcl_out = filtered->size();
        doNotOptimize(pcl_out);
      }
      pcl_time = total / ITERATIONS;
      printResult("[PCL] PassThrough", pcl_time, NUM_POINTS, pcl_out);
    }

    // nanoPCL cropZaxis (copy)
    {
      double total = 0;
      for (int iter = 0; iter < ITERATIONS; ++iter) {
        gen.seed(42 + iter);

        nanopcl::PointCloud cloud("benchmark");
        cloud.reserve(NUM_POINTS);
        cloud.enableIntensity();
        for (size_t i = 0; i < NUM_POINTS; ++i) {
          cloud.add(nanopcl::Point(pos(gen), pos(gen), pos(gen)),
                    nanopcl::Intensity(val(gen)));
        }

        Timer t;
        auto filtered = nanopcl::filters::cropZaxis(cloud, Z_MIN, Z_MAX);
        total += t.elapsed_ms();
        nano_out = filtered.size();
        doNotOptimize(nano_out);
      }
      double nano_time = total / ITERATIONS;
      printResult("[nanoPCL] cropZaxis (copy)", nano_time, NUM_POINTS, nano_out, pcl_time);
    }

    // nanoPCL cropZaxis (move)
    {
      double total = 0;
      for (int iter = 0; iter < ITERATIONS; ++iter) {
        gen.seed(42 + iter);

        nanopcl::PointCloud cloud("benchmark");
        cloud.reserve(NUM_POINTS);
        cloud.enableIntensity();
        for (size_t i = 0; i < NUM_POINTS; ++i) {
          cloud.add(nanopcl::Point(pos(gen), pos(gen), pos(gen)),
                    nanopcl::Intensity(val(gen)));
        }

        Timer t;
        auto filtered =
            nanopcl::filters::cropZaxis(std::move(cloud), Z_MIN, Z_MAX);
        total += t.elapsed_ms();
        nano_out = filtered.size();
        doNotOptimize(nano_out);
      }
      double nano_time = total / ITERATIONS;
      printResult("[nanoPCL] cropZaxis (move)", nano_time, NUM_POINTS, nano_out, pcl_time);
    }
  }

  // =========================================================================
  // CROPBOX BENCHMARK
  // =========================================================================
  printHeader("3. CROP BOX FILTER ([-10,-10,-5] to [10,10,5])");

  for (size_t NUM_POINTS : point_counts) {
    std::cout << "\n--- " << NUM_POINTS << " points ---\n";
    std::cout << std::left << std::setw(30) << "Method" << std::right
              << std::setw(14) << "Time" << std::setw(14) << "Throughput"
              << std::setw(10) << "Output" << std::setw(8) << "Speedup"
              << "\n";
    std::cout << std::string(70, '-') << "\n";

    Eigen::Vector4f min_pt(-10, -10, -5, 1);
    Eigen::Vector4f max_pt(10, 10, 5, 1);
    nanopcl::Point min_p(-10, -10, -5);
    nanopcl::Point max_p(10, 10, 5);

    double pcl_time = 0;
    size_t pcl_out = 0, nano_out = 0;

    // PCL CropBox
    {
      double total = 0;
      for (int iter = 0; iter < ITERATIONS; ++iter) {
        gen.seed(42 + iter);

        auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud->width = NUM_POINTS;
        cloud->height = 1;
        cloud->is_dense = true;
        cloud->points.resize(NUM_POINTS);
        for (size_t i = 0; i < NUM_POINTS; ++i) {
          cloud->points[i].x = pos(gen);
          cloud->points[i].y = pos(gen);
          cloud->points[i].z = pos(gen);
          cloud->points[i].intensity = val(gen);
        }

        auto filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::CropBox<pcl::PointXYZI> cb;
        cb.setInputCloud(cloud);
        cb.setMin(min_pt);
        cb.setMax(max_pt);

        Timer t;
        cb.filter(*filtered);
        total += t.elapsed_ms();
        pcl_out = filtered->size();
        doNotOptimize(pcl_out);
      }
      pcl_time = total / ITERATIONS;
      printResult("[PCL] CropBox", pcl_time, NUM_POINTS, pcl_out);
    }

    // nanoPCL cropBox (copy)
    {
      double total = 0;
      for (int iter = 0; iter < ITERATIONS; ++iter) {
        gen.seed(42 + iter);

        nanopcl::PointCloud cloud("benchmark");
        cloud.reserve(NUM_POINTS);
        cloud.enableIntensity();
        for (size_t i = 0; i < NUM_POINTS; ++i) {
          cloud.add(nanopcl::Point(pos(gen), pos(gen), pos(gen)),
                    nanopcl::Intensity(val(gen)));
        }

        Timer t;
        auto filtered = nanopcl::filters::cropBox(cloud, min_p, max_p);
        total += t.elapsed_ms();
        nano_out = filtered.size();
        doNotOptimize(nano_out);
      }
      double nano_time = total / ITERATIONS;
      printResult("[nanoPCL] cropBox (copy)", nano_time, NUM_POINTS, nano_out, pcl_time);
    }

    // nanoPCL cropBox (move)
    {
      double total = 0;
      for (int iter = 0; iter < ITERATIONS; ++iter) {
        gen.seed(42 + iter);

        nanopcl::PointCloud cloud("benchmark");
        cloud.reserve(NUM_POINTS);
        cloud.enableIntensity();
        for (size_t i = 0; i < NUM_POINTS; ++i) {
          cloud.add(nanopcl::Point(pos(gen), pos(gen), pos(gen)),
                    nanopcl::Intensity(val(gen)));
        }

        Timer t;
        auto filtered =
            nanopcl::filters::cropBox(std::move(cloud), min_p, max_p);
        total += t.elapsed_ms();
        nano_out = filtered.size();
        doNotOptimize(nano_out);
      }
      double nano_time = total / ITERATIONS;
      printResult("[nanoPCL] cropBox (move)", nano_time, NUM_POINTS, nano_out, pcl_time);
    }
  }

  // =========================================================================
  // RADIUS OUTLIER REMOVAL BENCHMARK
  // =========================================================================
  printHeader("4. RADIUS OUTLIER REMOVAL (radius=1.0, min_neighbors=5)");

  // Use smaller point counts for this expensive operation
  std::vector<size_t> ror_point_counts = {10000, 50000, 100000};

  for (size_t NUM_POINTS : ror_point_counts) {
    std::cout << "\n--- " << NUM_POINTS << " points ---\n";
    std::cout << std::left << std::setw(30) << "Method" << std::right
              << std::setw(14) << "Time" << std::setw(14) << "Throughput"
              << std::setw(10) << "Output" << std::setw(8) << "Speedup"
              << "\n";
    std::cout << std::string(70, '-') << "\n";

    const float RADIUS = 1.0f;
    const int MIN_NEIGHBORS = 5;
    double pcl_time = 0;
    size_t pcl_out = 0, nano_out = 0;

    // PCL RadiusOutlierRemoval
    {
      double total = 0;
      for (int iter = 0; iter < 5; ++iter) { // Fewer iterations for expensive op
        gen.seed(42 + iter);

        auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        cloud->width = NUM_POINTS;
        cloud->height = 1;
        cloud->is_dense = true;
        cloud->points.resize(NUM_POINTS);
        for (size_t i = 0; i < NUM_POINTS; ++i) {
          cloud->points[i].x = pos(gen);
          cloud->points[i].y = pos(gen);
          cloud->points[i].z = pos(gen);
          cloud->points[i].intensity = val(gen);
        }

        auto filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
        ror.setInputCloud(cloud);
        ror.setRadiusSearch(RADIUS);
        ror.setMinNeighborsInRadius(MIN_NEIGHBORS);

        Timer t;
        ror.filter(*filtered);
        total += t.elapsed_ms();
        pcl_out = filtered->size();
        doNotOptimize(pcl_out);
      }
      pcl_time = total / 5;
      printResult("[PCL] RadiusOutlierRemoval", pcl_time, NUM_POINTS, pcl_out);
    }

    // nanoPCL radiusOutlierRemoval
    {
      double total = 0;
      for (int iter = 0; iter < 5; ++iter) {
        gen.seed(42 + iter);

        nanopcl::PointCloud cloud("benchmark");
        cloud.reserve(NUM_POINTS);
        cloud.enableIntensity();
        for (size_t i = 0; i < NUM_POINTS; ++i) {
          cloud.add(nanopcl::Point(pos(gen), pos(gen), pos(gen)),
                    nanopcl::Intensity(val(gen)));
        }

        Timer t;
        auto filtered =
            nanopcl::filters::radiusOutlierRemoval(cloud, RADIUS, MIN_NEIGHBORS);
        total += t.elapsed_ms();
        nano_out = filtered.size();
        doNotOptimize(nano_out);
      }
      double nano_time = total / 5;
      printResult("[nanoPCL] radiusOutlierRemoval", nano_time, NUM_POINTS, nano_out, pcl_time);
    }
  }

  // Summary
  printHeader("SUMMARY");
  std::cout << R"(
Comparison notes:
- PCL uses AoS (Array of Structures) layout with 32 bytes per PointXYZI
- nanoPCL uses SoA (Structure of Arrays) layout with 16 bytes per point+intensity
- nanoPCL move version reuses input buffer (optimal performance)
- Speedup > 1.0x means nanoPCL is faster than PCL
)";

  return 0;
}
