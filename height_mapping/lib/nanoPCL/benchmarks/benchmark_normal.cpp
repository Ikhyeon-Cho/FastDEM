// nanoPCL vs PCL Benchmark
#include <iostream>
#include <vector>
#include <random>
#include <chrono>

// nanoPCL
#include <nanopcl/nanopcl.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

int main() {
  const int num_points = 100000;
  const float radius = 0.1f;

  std::cout << "Benchmark: 100k points, radius=" << radius << "\n";
  std::cout << "--------------------------------------------------\n";

  // 1. Generate Random Data
  nanopcl::PointCloud nano_cloud("bench");
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl_cloud->width = num_points;
  pcl_cloud->height = 1;
  pcl_cloud->points.resize(num_points);

  std::mt19937 gen(12345); // Fixed seed
  std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
  
  for (int i = 0; i < num_points; ++i) {
    float x = dist(gen);
    float y = dist(gen);
    float z = dist(gen) * 0.1f; // Flattened
    
    // Fill nanoPCL
    nano_cloud.push_back(nanopcl::Point(x, y, z));
    
    // Fill PCL
    pcl_cloud->points[i].x = x;
    pcl_cloud->points[i].y = y;
    pcl_cloud->points[i].z = z;
  }

  // --------------------------------------------------
  // 2. Run nanoPCL
  // --------------------------------------------------
  {
    std::vector<nanopcl::Point> normals;
    nanopcl::geometry::NormalEstimationConfig config;
    config.radius = radius;
    config.min_neighbors = 5;

    // Searcher setup time included as part of "total process"
    auto start = std::chrono::high_resolution_clock::now();
    
    nanopcl::search::VoxelHashMap searcher(config.radius);
    searcher.setInputCloud(nano_cloud);
    nanopcl::geometry::estimateNormals(nano_cloud, searcher, normals, config);
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "[nanoPCL] Time: " << elapsed.count() << " ms\n";
  }

  // --------------------------------------------------
  // 3. Run PCL
  // --------------------------------------------------
  {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    auto start = std::chrono::high_resolution_clock::now();

    ne.setInputCloud(pcl_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);
    ne.compute(*cloud_normals);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "[PCL]     Time: " << elapsed.count() << " ms\n";
  }

  return 0;
}
