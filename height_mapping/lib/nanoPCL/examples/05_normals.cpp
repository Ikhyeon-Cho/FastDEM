// nanoPCL Normal Estimation Example
#include <nanopcl/nanopcl.hpp>
#include <iostream>
#include <vector>
#include <random>
#include <chrono>

int main() {
  using namespace nanopcl;

  // 1. Generate a plane (z = 0)
  PointCloud cloud("plane");
  
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> dist(-5.0f, 5.0f);
  std::normal_distribution<float> noise(0.0f, 0.05f); // 5cm noise

  for (int i = 0; i < 100000; ++i) {
    float x = dist(gen);
    float y = dist(gen);
    float z = noise(gen); // Should be roughly 0
    cloud.push_back(Point(x, y, z));
  }

  std::cout << "Generated " << cloud.size() << " points on a noisy plane (z=0)\n";

  // 2. Estimate Normals
  std::vector<Point> normals;
  geometry::NormalEstimationConfig config;
  config.radius = 0.1f;

  auto start = std::chrono::high_resolution_clock::now();
  
  geometry::estimateNormals(cloud, normals, config);
  
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::milli> elapsed = end - start;

  std::cout << "Normal estimation took: " << elapsed.count() << " ms\n";

  // 3. Check results
  // Plane normal should be (0, 0, 1) or (0, 0, -1) depending on viewpoint
  // Here viewpoint is (0,0,0), points are around z=0.
  // We flip towards viewpoint, so if z < 0, normal points up?
  // Let's check the average error from Unit Z (0, 0, 1)
  
  float error_sum = 0.0f;
  int valid_count = 0;
  
  for (const auto& n : normals) {
    if (n.isZero()) continue; // Skipped points (not enough neighbors) 
    
    // Dot product with Z axis should be close to 1.0 (or -1.0)
    float dot = std::abs(n.dot(Point::UnitZ()));
    error_sum += (1.0f - dot);
    valid_count++;
  }

  std::cout << "Valid normals computed: " << valid_count << "\n";
  if (valid_count > 0) {
    std::cout << "Average angular error (dot prod dev): " << error_sum / valid_count << "\n";
  }

  // Check a few samples
  std::cout << "\nSample normals:\n";
  for (int i = 0; i < 5; ++i) {
    std::cout << "Point: " << cloud[i].transpose() 
              << " -> Normal: " << normals[i].transpose() << "\n";
  }

  return 0;
}
