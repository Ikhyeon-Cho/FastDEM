// nanoPCL vs PCL Benchmark: Normal Estimation
// Tests various scenarios: point counts, radii, density

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

// nanoPCL
#include <nanopcl/common.hpp>
#include <nanopcl/geometry/local_surface.hpp>

// PCL
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>

// =============================================================================
// Utilities
// =============================================================================

struct Stats {
  double mean;
  double stddev;
  double min;
  double max;
};

Stats computeStats(const std::vector<double>& data) {
  if (data.empty()) return {0, 0, 0, 0};

  double sum = std::accumulate(data.begin(), data.end(), 0.0);
  double mean = sum / data.size();

  double sq_sum = 0;
  for (double v : data) {
    sq_sum += (v - mean) * (v - mean);
  }
  double stddev = std::sqrt(sq_sum / data.size());

  auto [min_it, max_it] = std::minmax_element(data.begin(), data.end());
  return {mean, stddev, *min_it, *max_it};
}

// =============================================================================
// Data Generation
// =============================================================================

void generateData(int num_points, float range, float z_scale,
                  npcl::PointCloud& nano_cloud,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
  nano_cloud.clear();
  nano_cloud.reserve(num_points);

  pcl_cloud->clear();
  pcl_cloud->width = num_points;
  pcl_cloud->height = 1;
  pcl_cloud->points.resize(num_points);

  std::mt19937 gen(12345);
  std::uniform_real_distribution<float> xy_dist(-range, range);
  std::uniform_real_distribution<float> z_dist(-range * z_scale,
                                                range * z_scale);

  for (int i = 0; i < num_points; ++i) {
    float x = xy_dist(gen);
    float y = xy_dist(gen);
    float z = z_dist(gen);

    nano_cloud.add(npcl::Point(x, y, z));

    pcl_cloud->points[i].x = x;
    pcl_cloud->points[i].y = y;
    pcl_cloud->points[i].z = z;
  }
}

// =============================================================================
// Benchmark Functions
// =============================================================================

double benchNanoPCL(npcl::PointCloud cloud, float radius) {
  auto start = std::chrono::high_resolution_clock::now();
  npcl::geometry::estimateNormals(cloud, radius);  // In-place
  auto end = std::chrono::high_resolution_clock::now();

  // Prevent optimization
  volatile size_t count = cloud.normal().size();
  (void)count;

  return std::chrono::duration<double, std::milli>(end - start).count();
}

double benchPCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius) {
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());

  auto start = std::chrono::high_resolution_clock::now();

  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(radius);
  ne.compute(*normals);

  auto end = std::chrono::high_resolution_clock::now();

  // Prevent optimization
  volatile size_t count = normals->size();
  (void)count;

  return std::chrono::duration<double, std::milli>(end - start).count();
}

// =============================================================================
// Test Runner
// =============================================================================

void runScenario(const std::string& name, int num_points, float range,
                 float z_scale, float radius, int rounds) {
  npcl::PointCloud nano_cloud("bench");
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  generateData(num_points, range, z_scale, nano_cloud, pcl_cloud);

  std::vector<double> nano_times, pcl_times;

  // Warmup
  benchNanoPCL(nano_cloud, radius);
  benchPCL(pcl_cloud, radius);

  // Benchmark rounds
  for (int r = 0; r < rounds; ++r) {
    nano_times.push_back(benchNanoPCL(nano_cloud, radius));
    pcl_times.push_back(benchPCL(pcl_cloud, radius));
  }

  Stats nano_stats = computeStats(nano_times);
  Stats pcl_stats = computeStats(pcl_times);

  double speedup = pcl_stats.mean / nano_stats.mean;

  std::cout << std::left << std::setw(35) << name << std::right << std::fixed
            << std::setprecision(1) << std::setw(10) << nano_stats.mean
            << std::setw(10) << pcl_stats.mean << std::setw(10) << speedup
            << "x\n";
}

// =============================================================================
// Main
// =============================================================================

int main() {
  std::cout << "nanoPCL vs PCL: Normal Estimation Benchmark\n";
  std::cout << std::string(65, '=') << "\n\n";

  const int ROUNDS = 5;

  std::cout << std::left << std::setw(35) << "Scenario" << std::right
            << std::setw(10) << "nano(ms)" << std::setw(10) << "PCL(ms)"
            << std::setw(10) << "Speedup"
            << "\n";
  std::cout << std::string(65, '-') << "\n";

  // Varying point count (fixed radius=0.1, planar)
  std::cout << "\n[Varying Point Count] radius=0.1, planar\n";
  runScenario("50K points", 50000, 10.0f, 0.1f, 0.1f, ROUNDS);
  runScenario("100K points", 100000, 10.0f, 0.1f, 0.1f, ROUNDS);
  runScenario("200K points", 200000, 10.0f, 0.1f, 0.1f, ROUNDS);
  runScenario("500K points", 500000, 10.0f, 0.1f, 0.1f, ROUNDS);

  // Varying radius (fixed 100K points, planar)
  std::cout << "\n[Varying Radius] 100K points, planar\n";
  runScenario("radius=0.05 (sparse)", 100000, 10.0f, 0.1f, 0.05f, ROUNDS);
  runScenario("radius=0.1 (normal)", 100000, 10.0f, 0.1f, 0.1f, ROUNDS);
  runScenario("radius=0.2 (dense)", 100000, 10.0f, 0.1f, 0.2f, ROUNDS);
  runScenario("radius=0.5 (very dense)", 100000, 10.0f, 0.1f, 0.5f, ROUNDS);

  // Varying geometry (fixed 100K points, radius=0.1)
  std::cout << "\n[Varying Geometry] 100K points, radius=0.1\n";
  runScenario("Planar (z_scale=0.1)", 100000, 10.0f, 0.1f, 0.1f, ROUNDS);
  runScenario("Moderate (z_scale=0.5)", 100000, 10.0f, 0.5f, 0.1f, ROUNDS);
  runScenario("3D Uniform (z_scale=1.0)", 100000, 10.0f, 1.0f, 0.1f, ROUNDS);

  // Real-world scenarios
  std::cout << "\n[Real-world Scenarios]\n";
  runScenario("Indoor (50K, r=0.05)", 50000, 5.0f, 0.3f, 0.05f, ROUNDS);
  runScenario("Outdoor (200K, r=0.2)", 200000, 25.0f, 0.2f, 0.2f, ROUNDS);
  runScenario("Dense mapping (100K, r=0.1)", 100000, 10.0f, 0.1f, 0.1f, ROUNDS);

  std::cout << "\n" << std::string(65, '=') << "\n";
  std::cout << "Note: Speedup = PCL_time / nanoPCL_time\n";

  return 0;
}
