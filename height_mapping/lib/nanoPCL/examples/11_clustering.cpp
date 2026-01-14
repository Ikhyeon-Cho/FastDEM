// nanoPCL Example 11: Euclidean Clustering
//
// Demonstrates point cloud clustering for obstacle detection.
// Typical pipeline: Ground removal -> Clustering -> Object processing

#include <cmath>
#include <iomanip>
#include <iostream>
#include <nanopcl/common.hpp>
#include <nanopcl/segmentation.hpp>
#include <random>

using namespace npcl;

// Helper: Generate synthetic LiDAR scene with ground and objects
PointCloud generateScene() {
  PointCloud cloud("base_link");
  cloud.reserve(10000);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> noise(-0.02f, 0.02f);

  // 1. Ground plane (z ~ 0)
  std::uniform_real_distribution<float> ground_xy(-15.0f, 15.0f);
  for (int i = 0; i < 5000; ++i) {
    float x = ground_xy(gen);
    float y = ground_xy(gen);
    float z = noise(gen);  // Ground with slight noise
    cloud.add(Point(x, y, z));
  }

  // 2. Object 1: Car-like box at (5, 3, 0.8)
  std::uniform_real_distribution<float> car_x(3.0f, 7.0f);
  std::uniform_real_distribution<float> car_y(1.5f, 4.5f);
  std::uniform_real_distribution<float> car_z(0.2f, 1.5f);
  for (int i = 0; i < 800; ++i) {
    cloud.add(Point(car_x(gen), car_y(gen), car_z(gen)));
  }

  // 3. Object 2: Pedestrian at (-3, -2, 0.9)
  std::normal_distribution<float> ped_xy(0.0f, 0.3f);
  std::uniform_real_distribution<float> ped_z(0.2f, 1.7f);
  for (int i = 0; i < 200; ++i) {
    float x = -3.0f + ped_xy(gen);
    float y = -2.0f + ped_xy(gen);
    float z = ped_z(gen);
    cloud.add(Point(x, y, z));
  }

  // 4. Object 3: Tree at (8, -5, 2.5)
  std::normal_distribution<float> tree_xy(0.0f, 0.8f);
  std::uniform_real_distribution<float> tree_z(0.3f, 4.0f);
  for (int i = 0; i < 600; ++i) {
    float x = 8.0f + tree_xy(gen);
    float y = -5.0f + tree_xy(gen);
    float z = tree_z(gen);
    cloud.add(Point(x, y, z));
  }

  // 5. Scattered noise points
  std::uniform_real_distribution<float> rand_pos(-10.0f, 10.0f);
  std::uniform_real_distribution<float> rand_z(0.0f, 5.0f);
  for (int i = 0; i < 100; ++i) {
    cloud.add(Point(rand_pos(gen), rand_pos(gen), rand_z(gen)));
  }

  return cloud;
}

int main() {
  std::cout << "=== nanoPCL Euclidean Clustering Example ===\n\n";

  // ===========================================================================
  // 1. Generate Scene
  // ===========================================================================
  std::cout << "1. Generating synthetic LiDAR scene...\n";
  PointCloud cloud = generateScene();
  std::cout << "   Total points: " << cloud.size() << "\n\n";

  // ===========================================================================
  // 2. Ground Removal (RANSAC)
  // ===========================================================================
  std::cout << "2. Ground removal using RANSAC...\n";

  auto ground_result =
      segmentation::segmentPlane(cloud, 0.1f);  // 10cm threshold
  std::cout << "   Ground points (inliers):  " << ground_result.inliers.size()
            << "\n";

  auto obstacle_indices = ground_result.outliers(cloud.size());
  std::cout << "   Obstacle points (outliers): " << obstacle_indices.size()
            << "\n\n";

  // ===========================================================================
  // 3. Euclidean Clustering
  // ===========================================================================
  std::cout << "3. Clustering obstacles...\n";

  segmentation::ClusterConfig config;
  config.tolerance = 0.5f;  // 50cm neighbor distance
  config.min_size = 20;     // Minimum 20 points per cluster
  config.max_size = 5000;   // Maximum 5000 points per cluster

  // Cluster only the obstacle points (not ground)
  auto clusters =
      segmentation::euclideanCluster(cloud, obstacle_indices, config);

  std::cout << "   Clusters found: " << clusters.numClusters() << "\n";
  std::cout << "   Clustered points: " << clusters.totalClusteredPoints()
            << "\n";
  std::cout << "   Noise points: "
            << (obstacle_indices.size() - clusters.totalClusteredPoints())
            << "\n\n";

  // ===========================================================================
  // 4. Analyze Each Cluster
  // ===========================================================================
  std::cout << "4. Cluster analysis:\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "   " << std::setw(8) << "Cluster" << std::setw(8) << "Points"
            << std::setw(10) << "Center X" << std::setw(10) << "Center Y"
            << std::setw(10) << "Center Z" << std::setw(10) << "Height"
            << "\n";
  std::cout << "   " << std::string(56, '-') << "\n";

  for (size_t i = 0; i < clusters.numClusters(); ++i) {
    // Extract cluster as separate point cloud
    PointCloud obj = clusters.extract(cloud, i);

    // Compute centroid
    Point centroid(0, 0, 0);
    float z_min = std::numeric_limits<float>::max();
    float z_max = std::numeric_limits<float>::lowest();

    for (size_t j = 0; j < obj.size(); ++j) {
      centroid += obj[j];
      z_min = std::min(z_min, obj[j].z());
      z_max = std::max(z_max, obj[j].z());
    }
    centroid /= static_cast<float>(obj.size());

    float height = z_max - z_min;

    std::cout << "   " << std::setw(8) << i << std::setw(8) << obj.size()
              << std::setw(10) << centroid.x() << std::setw(10) << centroid.y()
              << std::setw(10) << centroid.z() << std::setw(10) << height
              << "\n";
  }
  std::cout << "\n";

  // ===========================================================================
  // 5. Apply Labels for Visualization
  // ===========================================================================
  std::cout << "5. Applying cluster labels...\n";

  // Method A: Apply to original cloud (indices reference original cloud)
  segmentation::applyClusterLabels(cloud, clusters);

  // Verify labels on original cloud
  size_t labeled_count = 0;
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (cloud.label()[i].instanceId() > 0) {
      labeled_count++;
    }
  }
  std::cout << "   Points with cluster labels: " << labeled_count << "\n";
  std::cout << "   Points marked as noise (id=0): "
            << (cloud.size() - labeled_count) << "\n";

  // Method B: Cluster extracted obstacles directly (for independent processing)
  std::vector<size_t> size_t_indices(obstacle_indices.begin(),
                                     obstacle_indices.end());
  PointCloud obstacles = cloud[size_t_indices];

  // Cluster the extracted cloud directly (indices are 0-based within obstacles)
  auto obstacle_clusters = segmentation::euclideanCluster(obstacles, config);
  segmentation::applyClusterLabels(obstacles, obstacle_clusters);

  size_t obs_labeled = 0;
  for (size_t i = 0; i < obstacles.size(); ++i) {
    if (obstacles.label()[i].instanceId() > 0) {
      obs_labeled++;
    }
  }
  std::cout << "   (Alternative) Obstacles labeled: " << obs_labeled << "\n\n";

  // ===========================================================================
  // 6. Direct Iteration (CSR Access)
  // ===========================================================================
  std::cout << "6. Direct CSR access (no extraction):\n";

  // Process cluster 0 without creating a new PointCloud
  if (clusters.numClusters() > 0) {
    Point centroid(0, 0, 0);
    for (uint32_t idx : clusters.clusterIndices(0)) {
      centroid += cloud[idx];
    }
    centroid /= static_cast<float>(clusters.clusterSize(0));
    std::cout << "   Cluster 0 centroid (direct): (" << centroid.x() << ", "
              << centroid.y() << ", " << centroid.z() << ")\n";
  }

  std::cout << "\nDone!\n";
  return 0;
}
