// Benchmark: nanoPCL vs PCL RANSAC - Accuracy Comparison
//
// Measures:
//   - Normal estimation accuracy (angle error vs ground truth)
//   - Distance parameter accuracy (d value error)
//   - Inlier detection accuracy (precision, recall, F1)
//
// Build:
//   g++ -O3 -march=native -fopenmp -std=c++17 bench_ransac_accuracy.cpp \
//       -o bench_ransac_accuracy \
//       -I../../include -I/usr/include/eigen3 \
//       -I/usr/include/pcl-1.10 \
//       $(pkg-config --libs pcl_segmentation-1.10 pcl_common-1.10 pcl_sample_consensus-1.10)

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <set>
#include <vector>

// nanoPCL
#include "nanopcl/core/point_cloud.hpp"
#include "nanopcl/segmentation/ransac_plane.hpp"

// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// =============================================================================
// Ground Truth Plane
// =============================================================================

struct GroundTruthPlane {
  Eigen::Vector3f normal;
  float d;
  std::set<size_t> inlier_indices;
};

// =============================================================================
// Test Data Generation with Ground Truth
// =============================================================================

GroundTruthPlane generateTestData(size_t n_ground, size_t n_obstacles,
                                   float noise,
                                   npcl::PointCloud& nano_cloud,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
  std::mt19937 rng(42);
  std::uniform_real_distribution<float> xy_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> noise_dist(-noise, noise);
  std::uniform_real_distribution<float> z_dist(0.5f, 3.0f);

  size_t total = n_ground + n_obstacles;

  // Ground truth: z = 0 plane (normal = [0, 0, 1], d = 0)
  GroundTruthPlane gt;
  gt.normal = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
  gt.d = 0.0f;

  nano_cloud.clear();
  nano_cloud.reserve(total);

  pcl_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl_cloud->reserve(total);

  // Ground plane points (z ≈ 0, these are true inliers)
  for (size_t i = 0; i < n_ground; ++i) {
    float x = xy_dist(rng);
    float y = xy_dist(rng);
    float z = noise_dist(rng);  // z close to 0
    nano_cloud.add(npcl::Point(x, y, z));
    pcl_cloud->push_back(pcl::PointXYZ(x, y, z));
    gt.inlier_indices.insert(i);
  }

  // Obstacle points (z > 0.5, these are true outliers)
  for (size_t i = 0; i < n_obstacles; ++i) {
    float x = xy_dist(rng);
    float y = xy_dist(rng);
    float z = z_dist(rng);
    nano_cloud.add(npcl::Point(x, y, z));
    pcl_cloud->push_back(pcl::PointXYZ(x, y, z));
  }

  return gt;
}

// =============================================================================
// Accuracy Metrics
// =============================================================================

struct AccuracyMetrics {
  double normal_angle_error;  // degrees
  double d_error;             // absolute error
  double precision;           // TP / (TP + FP)
  double recall;              // TP / (TP + FN)
  double f1_score;            // 2 * precision * recall / (precision + recall)
  size_t true_positives;
  size_t false_positives;
  size_t false_negatives;
};

AccuracyMetrics computeAccuracy(const GroundTruthPlane& gt,
                                 const Eigen::Vector3f& estimated_normal,
                                 float estimated_d,
                                 const std::vector<uint32_t>& detected_inliers,
                                 size_t total_points) {
  AccuracyMetrics metrics{};

  // Normal angle error (handle sign ambiguity)
  float dot = std::abs(gt.normal.dot(estimated_normal));
  dot = std::min(dot, 1.0f);  // Clamp for numerical stability
  metrics.normal_angle_error = std::acos(dot) * 180.0 / M_PI;

  // D error (handle sign ambiguity based on normal direction)
  float sign = (gt.normal.dot(estimated_normal) > 0) ? 1.0f : -1.0f;
  metrics.d_error = std::abs(gt.d - sign * estimated_d);

  // Inlier detection accuracy
  std::set<size_t> detected_set(detected_inliers.begin(), detected_inliers.end());

  for (size_t idx : detected_inliers) {
    if (gt.inlier_indices.count(idx)) {
      metrics.true_positives++;
    } else {
      metrics.false_positives++;
    }
  }

  metrics.false_negatives = gt.inlier_indices.size() - metrics.true_positives;

  // Precision, Recall, F1
  if (metrics.true_positives + metrics.false_positives > 0) {
    metrics.precision = static_cast<double>(metrics.true_positives) /
                        (metrics.true_positives + metrics.false_positives);
  }

  if (metrics.true_positives + metrics.false_negatives > 0) {
    metrics.recall = static_cast<double>(metrics.true_positives) /
                     (metrics.true_positives + metrics.false_negatives);
  }

  if (metrics.precision + metrics.recall > 0) {
    metrics.f1_score =
        2.0 * metrics.precision * metrics.recall / (metrics.precision + metrics.recall);
  }

  return metrics;
}

// =============================================================================
// Run Tests
// =============================================================================

int main() {
  std::cout << "=== nanoPCL vs PCL: Accuracy Comparison ===" << std::endl;
  std::cout << std::endl;

  // Parameters
  float threshold = 0.1f;
  int max_iters = 1000;

  // Test configurations: (ground_points, obstacle_points, noise_level)
  std::vector<std::tuple<size_t, size_t, float, std::string>> configs = {
      {80000, 20000, 0.02f, "Low noise (2cm)"},
      {80000, 20000, 0.05f, "Medium noise (5cm)"},
      {80000, 20000, 0.08f, "High noise (8cm)"},
      {50000, 50000, 0.02f, "50% outliers"},
      {30000, 70000, 0.02f, "70% outliers"},
  };

  std::cout << std::fixed << std::setprecision(4);

  for (const auto& [n_ground, n_obstacles, noise, desc] : configs) {
    std::cout << "=== " << desc << " (Ground: " << n_ground
              << ", Obstacles: " << n_obstacles << ") ===" << std::endl;

    // Generate data
    npcl::PointCloud nano_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud;
    auto gt = generateTestData(n_ground, n_obstacles, noise, nano_cloud, pcl_cloud);

    // =========================================================================
    // nanoPCL
    // =========================================================================
    auto nano_result =
        npcl::segmentation::segmentPlane(nano_cloud, threshold, max_iters, 0.99);

    auto nano_metrics =
        computeAccuracy(gt, nano_result.model.normal, nano_result.model.d,
                        nano_result.inliers, nano_cloud.size());

    // =========================================================================
    // PCL
    // =========================================================================
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr pcl_inliers(new pcl::PointIndices());

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iters);
    seg.setDistanceThreshold(threshold);
    seg.setInputCloud(pcl_cloud);
    seg.segment(*pcl_inliers, *coefficients);

    Eigen::Vector3f pcl_normal(coefficients->values[0], coefficients->values[1],
                               coefficients->values[2]);
    float pcl_d = coefficients->values[3];

    std::vector<uint32_t> pcl_inlier_vec(pcl_inliers->indices.begin(),
                                          pcl_inliers->indices.end());

    auto pcl_metrics =
        computeAccuracy(gt, pcl_normal, pcl_d, pcl_inlier_vec, pcl_cloud->size());

    // =========================================================================
    // Print Results
    // =========================================================================
    std::cout << std::endl;
    std::cout << "                        nanoPCL         PCL" << std::endl;
    std::cout << "  ------------------------------------------------" << std::endl;
    std::cout << "  Normal angle error:   " << std::setw(8) << nano_metrics.normal_angle_error
              << "°    " << std::setw(8) << pcl_metrics.normal_angle_error << "°" << std::endl;
    std::cout << "  Distance (d) error:   " << std::setw(8) << nano_metrics.d_error
              << "m    " << std::setw(8) << pcl_metrics.d_error << "m" << std::endl;
    std::cout << "  Precision:            " << std::setw(8) << nano_metrics.precision * 100
              << "%   " << std::setw(8) << pcl_metrics.precision * 100 << "%" << std::endl;
    std::cout << "  Recall:               " << std::setw(8) << nano_metrics.recall * 100
              << "%   " << std::setw(8) << pcl_metrics.recall * 100 << "%" << std::endl;
    std::cout << "  F1 Score:             " << std::setw(8) << nano_metrics.f1_score * 100
              << "%   " << std::setw(8) << pcl_metrics.f1_score * 100 << "%" << std::endl;
    std::cout << "  Detected inliers:     " << std::setw(8) << nano_result.inliers.size()
              << "     " << std::setw(8) << pcl_inliers->indices.size() << std::endl;
    std::cout << "  True positives:       " << std::setw(8) << nano_metrics.true_positives
              << "     " << std::setw(8) << pcl_metrics.true_positives << std::endl;
    std::cout << std::endl;
  }

  // ===========================================================================
  // Statistical Test (Multiple Runs)
  // ===========================================================================
  std::cout << "=== Statistical Comparison (20 runs, 100K points) ===" << std::endl;
  std::cout << std::endl;

  std::vector<double> nano_angles, pcl_angles;
  std::vector<double> nano_f1s, pcl_f1s;

  for (int run = 0; run < 20; ++run) {
    // Different seed for each run
    std::mt19937 rng(run * 1000);

    npcl::PointCloud nano_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud;

    // Regenerate with different seed (inline generation)
    nano_cloud.reserve(100000);
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_cloud->reserve(100000);

    std::uniform_real_distribution<float> xy_dist(-50.0f, 50.0f);
    std::uniform_real_distribution<float> noise_dist(-0.02f, 0.02f);
    std::uniform_real_distribution<float> z_dist(0.5f, 3.0f);

    std::set<size_t> gt_inliers;

    for (size_t i = 0; i < 80000; ++i) {
      float x = xy_dist(rng);
      float y = xy_dist(rng);
      float z = noise_dist(rng);
      nano_cloud.add(npcl::Point(x, y, z));
      pcl_cloud->push_back(pcl::PointXYZ(x, y, z));
      gt_inliers.insert(i);
    }

    for (size_t i = 0; i < 20000; ++i) {
      float x = xy_dist(rng);
      float y = xy_dist(rng);
      float z = z_dist(rng);
      nano_cloud.add(npcl::Point(x, y, z));
      pcl_cloud->push_back(pcl::PointXYZ(x, y, z));
    }

    GroundTruthPlane gt;
    gt.normal = Eigen::Vector3f(0, 0, 1);
    gt.d = 0;
    gt.inlier_indices = gt_inliers;

    // nanoPCL
    auto nano_res = npcl::segmentation::segmentPlane(nano_cloud, threshold, max_iters, 0.99);
    auto nano_m = computeAccuracy(gt, nano_res.model.normal, nano_res.model.d,
                                  nano_res.inliers, nano_cloud.size());
    nano_angles.push_back(nano_m.normal_angle_error);
    nano_f1s.push_back(nano_m.f1_score);

    // PCL
    pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inl(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(max_iters);
    seg.setDistanceThreshold(threshold);
    seg.setInputCloud(pcl_cloud);
    seg.segment(*inl, *coef);

    Eigen::Vector3f pcl_n(coef->values[0], coef->values[1], coef->values[2]);
    std::vector<uint32_t> pcl_inv(inl->indices.begin(), inl->indices.end());
    auto pcl_m = computeAccuracy(gt, pcl_n, coef->values[3], pcl_inv, pcl_cloud->size());
    pcl_angles.push_back(pcl_m.normal_angle_error);
    pcl_f1s.push_back(pcl_m.f1_score);
  }

  // Compute statistics
  auto mean = [](const std::vector<double>& v) {
    double sum = 0;
    for (double x : v) sum += x;
    return sum / v.size();
  };

  auto stddev = [&mean](const std::vector<double>& v) {
    double m = mean(v);
    double sum = 0;
    for (double x : v) sum += (x - m) * (x - m);
    return std::sqrt(sum / v.size());
  };

  std::cout << "                        nanoPCL              PCL" << std::endl;
  std::cout << "  --------------------------------------------------------" << std::endl;
  std::cout << "  Normal error (mean):  " << std::setw(8) << mean(nano_angles)
            << "°        " << std::setw(8) << mean(pcl_angles) << "°" << std::endl;
  std::cout << "  Normal error (std):   " << std::setw(8) << stddev(nano_angles)
            << "°        " << std::setw(8) << stddev(pcl_angles) << "°" << std::endl;
  std::cout << "  F1 Score (mean):      " << std::setw(8) << mean(nano_f1s) * 100
            << "%       " << std::setw(8) << mean(pcl_f1s) * 100 << "%" << std::endl;
  std::cout << "  F1 Score (std):       " << std::setw(8) << stddev(nano_f1s) * 100
            << "%       " << std::setw(8) << stddev(pcl_f1s) * 100 << "%" << std::endl;

  std::cout << std::endl;
  std::cout << "Benchmark complete." << std::endl;

  return 0;
}
