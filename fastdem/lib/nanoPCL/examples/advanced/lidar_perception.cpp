// nanoPCL Recipe: LiDAR Perception Pipeline
//
// Complete perception pipeline with step-by-step visualization:
// 1. VoxelGrid downsampling
// 2. RANSAC ground removal
// 3. Euclidean clustering
// 4. Bounding box extraction
//
// Each step saves a colored PCD file for visualization.
// View results with: pcl_viewer output/*.pcd
//
// Build: cmake .. -DNANOPCL_BUILD_EXAMPLES=ON && make recipe_lidar_perception
// Run:   ./recipe_lidar_perception

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <nanopcl/common.hpp>
#include <nanopcl/io.hpp>
#include <nanopcl/segmentation.hpp>
#include <random>

using namespace nanopcl;
namespace fs = std::filesystem;

// =============================================================================
// Timer Utility
// =============================================================================

class Timer {
public:
  Timer()
      : start_(std::chrono::high_resolution_clock::now()) {}

  double elapsedMs() const {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(now - start_).count();
  }

  void reset() { start_ = std::chrono::high_resolution_clock::now(); }

private:
  std::chrono::high_resolution_clock::time_point start_;
};

// =============================================================================
// Color Palette for Clusters
// =============================================================================

Color getClusterColor(size_t cluster_id) {
  // Distinct colors for up to 12 clusters, then cycle
  static const std::array<Color, 12> palette = {{
      Color(255, 0, 0),     // Red
      Color(0, 0, 255),     // Blue
      Color(255, 165, 0),   // Orange
      Color(128, 0, 128),   // Purple
      Color(0, 255, 255),   // Cyan
      Color(255, 0, 255),   // Magenta
      Color(255, 255, 0),   // Yellow
      Color(0, 128, 128),   // Teal
      Color(255, 192, 203), // Pink
      Color(165, 42, 42),   // Brown
      Color(0, 128, 0),     // Dark Green
      Color(70, 130, 180),  // Steel Blue
  }};
  return palette[cluster_id % palette.size()];
}

// =============================================================================
// Synthetic Scene Generator
// =============================================================================

PointCloud generateLidarScene() {
  PointCloud cloud("base_link");
  cloud.reserve(50000);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> noise(-0.03f, 0.03f);

  // 1. Ground plane (z ~ 0, simulating flat road)
  std::uniform_real_distribution<float> ground_x(-20.0f, 20.0f);
  std::uniform_real_distribution<float> ground_y(-15.0f, 15.0f);
  for (int i = 0; i < 20000; ++i) {
    float x = ground_x(gen);
    float y = ground_y(gen);
    float z = noise(gen); // Ground with measurement noise
    cloud.add(Point(x, y, z));
  }

  // 2. Vehicle 1: Car at (8, 2)
  std::uniform_real_distribution<float> car1_x(6.0f, 10.0f);
  std::uniform_real_distribution<float> car1_y(0.5f, 3.5f);
  std::uniform_real_distribution<float> car1_z(0.1f, 1.6f);
  for (int i = 0; i < 1500; ++i) {
    cloud.add(Point(car1_x(gen), car1_y(gen), car1_z(gen)));
  }

  // 3. Vehicle 2: Truck at (12, -4)
  std::uniform_real_distribution<float> truck_x(10.0f, 16.0f);
  std::uniform_real_distribution<float> truck_y(-6.0f, -2.0f);
  std::uniform_real_distribution<float> truck_z(0.1f, 2.8f);
  for (int i = 0; i < 2500; ++i) {
    cloud.add(Point(truck_x(gen), truck_y(gen), truck_z(gen)));
  }

  // 4. Pedestrian 1 at (-5, 3)
  std::normal_distribution<float> ped1_x(-5.0f, 0.25f);
  std::normal_distribution<float> ped1_y(3.0f, 0.25f);
  std::uniform_real_distribution<float> ped_z(0.1f, 1.8f);
  for (int i = 0; i < 300; ++i) {
    cloud.add(Point(ped1_x(gen), ped1_y(gen), ped_z(gen)));
  }

  // 5. Pedestrian 2 at (-3, -5)
  std::normal_distribution<float> ped2_x(-3.0f, 0.25f);
  std::normal_distribution<float> ped2_y(-5.0f, 0.25f);
  for (int i = 0; i < 280; ++i) {
    cloud.add(Point(ped2_x(gen), ped2_y(gen), ped_z(gen)));
  }

  // 6. Cyclist at (3, -8)
  std::normal_distribution<float> cyclist_x(3.0f, 0.4f);
  std::normal_distribution<float> cyclist_y(-8.0f, 0.6f);
  std::uniform_real_distribution<float> cyclist_z(0.1f, 1.5f);
  for (int i = 0; i < 400; ++i) {
    cloud.add(Point(cyclist_x(gen), cyclist_y(gen), cyclist_z(gen)));
  }

  // 7. Static object: Tree at (-12, 0)
  std::normal_distribution<float> tree_xy(0.0f, 0.6f);
  std::uniform_real_distribution<float> tree_z(0.2f, 4.5f);
  for (int i = 0; i < 800; ++i) {
    float x = -12.0f + tree_xy(gen);
    float y = 0.0f + tree_xy(gen);
    float z = tree_z(gen);
    cloud.add(Point(x, y, z));
  }

  // 8. Static object: Pole at (0, 10)
  std::normal_distribution<float> pole_xy(0.0f, 0.1f);
  std::uniform_real_distribution<float> pole_z(0.1f, 3.0f);
  for (int i = 0; i < 150; ++i) {
    float x = 0.0f + pole_xy(gen);
    float y = 10.0f + pole_xy(gen);
    float z = pole_z(gen);
    cloud.add(Point(x, y, z));
  }

  // 9. Sparse noise
  std::uniform_real_distribution<float> rand_x(-20.0f, 20.0f);
  std::uniform_real_distribution<float> rand_y(-15.0f, 15.0f);
  std::uniform_real_distribution<float> rand_z(0.0f, 5.0f);
  for (int i = 0; i < 200; ++i) {
    cloud.add(Point(rand_x(gen), rand_y(gen), rand_z(gen)));
  }

  return cloud;
}

// =============================================================================
// Bounding Box Structure
// =============================================================================

struct BoundingBox {
  Point min_pt;
  Point max_pt;
  Point centroid;
  size_t num_points;

  float width() const { return max_pt.x() - min_pt.x(); }
  float length() const { return max_pt.y() - min_pt.y(); }
  float height() const { return max_pt.z() - min_pt.z(); }
};

BoundingBox computeBBox(const PointCloud& cluster) {
  BoundingBox bbox;
  bbox.min_pt = Point(std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max(),
                      std::numeric_limits<float>::max());
  bbox.max_pt = Point(std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest(),
                      std::numeric_limits<float>::lowest());
  bbox.centroid = Point(0, 0, 0);
  bbox.num_points = cluster.size();

  for (size_t i = 0; i < cluster.size(); ++i) {
    const auto& p = cluster[i];
    bbox.min_pt.x() = std::min(bbox.min_pt.x(), p.x());
    bbox.min_pt.y() = std::min(bbox.min_pt.y(), p.y());
    bbox.min_pt.z() = std::min(bbox.min_pt.z(), p.z());
    bbox.max_pt.x() = std::max(bbox.max_pt.x(), p.x());
    bbox.max_pt.y() = std::max(bbox.max_pt.y(), p.y());
    bbox.max_pt.z() = std::max(bbox.max_pt.z(), p.z());
    bbox.centroid += p;
  }
  bbox.centroid /= static_cast<float>(cluster.size());

  return bbox;
}

// =============================================================================
// Benchmark Configuration
// =============================================================================

constexpr int WARMUP_RUNS = 10;    // System-level warm-up (CPU freq, caches)
constexpr int DISCARD_RUNS = 3;    // Discard first N benchmark runs
constexpr int BENCHMARK_RUNS = 10; // Actual measured runs

struct BenchmarkResult {
  double min_ms = std::numeric_limits<double>::max();
  double max_ms = 0;
  double total_ms = 0;
  int count = 0;

  void add(double ms) {
    min_ms = std::min(min_ms, ms);
    max_ms = std::max(max_ms, ms);
    total_ms += ms;
    ++count;
  }

  void reset() {
    min_ms = std::numeric_limits<double>::max();
    max_ms = 0;
    total_ms = 0;
    count = 0;
  }

  double avg() const { return count > 0 ? total_ms / count : 0; }
};

// =============================================================================
// Main Pipeline
// =============================================================================

int main() {
  std::cout
      << "╔════════════════════════════════════════════════════════════╗\n";
  std::cout
      << "║       nanoPCL LiDAR Perception Pipeline Demo               ║\n";
  std::cout
      << "╚════════════════════════════════════════════════════════════╝\n\n";

  // Create output directory
  const std::string output_dir = "perception_output";
  fs::create_directories(output_dir);
  std::cout << "Output directory: " << fs::absolute(output_dir) << "\n\n";

  Timer step_timer;

  // ===========================================================================
  // Step 1: Generate Input Data
  // ===========================================================================
  std::cout << "━━━ Step 1: Input Data ━━━\n";
  step_timer.reset();

  PointCloud input = generateLidarScene();
  double gen_time = step_timer.elapsedMs();

  std::cout << "  Points:     " << input.size() << "\n";
  std::cout << "  Frame:      " << input.frameId() << "\n";
  std::cout << "  Time:       " << std::fixed << std::setprecision(2)
            << gen_time << " ms\n";

  // Save original (white)
  PointCloud input_viz = input;
  input_viz.enableColor();
  for (size_t i = 0; i < input_viz.size(); ++i) {
    input_viz.color()[i] = Color::White();
  }
  io::savePCD(output_dir + "/01_input.pcd", input_viz);
  std::cout << "  Saved:      01_input.pcd\n\n";

  // ===========================================================================
  // Warm-up Phase (critical for stable benchmarking)
  // ===========================================================================
  std::cout << "━━━ Warm-up Phase (" << WARMUP_RUNS << " runs) ━━━\n";
  std::cout << "  Warming up CPU frequency and caches...\n";

  const float voxel_size = 0.1f;
  segmentation::ClusterConfig cluster_cfg;
  cluster_cfg.tolerance = 0.5f;
  cluster_cfg.min_size = 10;
  cluster_cfg.max_size = 10000;

  for (int i = 0; i < WARMUP_RUNS; ++i) {
    auto down = filters::voxelGrid(input, voxel_size);
    auto ground_res = segmentation::segmentPlane(down, 0.15f);
    auto obs_idx = ground_res.outliers(down.size());
    std::vector<size_t> obs_idx_st(obs_idx.begin(), obs_idx.end());
    auto obs = down[obs_idx_st];
    [[maybe_unused]] auto clust =
        segmentation::euclideanCluster(obs, cluster_cfg);
  }
  std::cout << "  Done.\n\n";

  // ===========================================================================
  // Benchmark Phase
  // ===========================================================================
  const int total_runs = DISCARD_RUNS + BENCHMARK_RUNS;
  std::cout << "━━━ Benchmark Phase (" << total_runs << " runs, first "
            << DISCARD_RUNS << " discarded) ━━━\n";

  BenchmarkResult voxel_bench, ransac_bench, cluster_bench, total_bench;

  PointCloud downsampled;
  segmentation::RansacResult ground_result;
  std::vector<uint32_t> obstacle_indices;
  PointCloud obstacles;
  segmentation::ClusterResult clusters;

  for (int run = 0; run < total_runs; ++run) {
    Timer run_timer;

    // VoxelGrid
    step_timer.reset();
    downsampled = filters::voxelGrid(input, voxel_size);
    double voxel_time = step_timer.elapsedMs();

    // RANSAC
    step_timer.reset();
    ground_result = segmentation::segmentPlane(downsampled, 0.15f);
    double ransac_time = step_timer.elapsedMs();

    obstacle_indices = ground_result.outliers(downsampled.size());
    std::vector<size_t> obs_idx(obstacle_indices.begin(),
                                obstacle_indices.end());
    obstacles = downsampled[obs_idx];

    // Clustering
    step_timer.reset();
    clusters = segmentation::euclideanCluster(obstacles, cluster_cfg);
    double cluster_time = step_timer.elapsedMs();

    double total_time = run_timer.elapsedMs();

    // Only record after discard phase
    if (run >= DISCARD_RUNS) {
      voxel_bench.add(voxel_time);
      ransac_bench.add(ransac_time);
      cluster_bench.add(cluster_time);
      total_bench.add(total_time);
    }
  }

  // Print benchmark results
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  ┌─────────────┬──────────┬──────────┬──────────┐\n";
  std::cout << "  │   Stage     │  Min(ms) │  Avg(ms) │  Max(ms) │\n";
  std::cout << "  ├─────────────┼──────────┼──────────┼──────────┤\n";
  std::cout << "  │ VoxelGrid   │ " << std::setw(8) << voxel_bench.min_ms
            << " │ " << std::setw(8) << voxel_bench.avg() << " │ "
            << std::setw(8) << voxel_bench.max_ms << " │\n";
  std::cout << "  │ RANSAC      │ " << std::setw(8) << ransac_bench.min_ms
            << " │ " << std::setw(8) << ransac_bench.avg() << " │ "
            << std::setw(8) << ransac_bench.max_ms << " │\n";
  std::cout << "  │ Clustering  │ " << std::setw(8) << cluster_bench.min_ms
            << " │ " << std::setw(8) << cluster_bench.avg() << " │ "
            << std::setw(8) << cluster_bench.max_ms << " │\n";
  std::cout << "  ├─────────────┼──────────┼──────────┼──────────┤\n";
  std::cout << "  │ TOTAL       │ " << std::setw(8) << total_bench.min_ms
            << " │ " << std::setw(8) << total_bench.avg() << " │ "
            << std::setw(8) << total_bench.max_ms << " │\n";
  std::cout << "  └─────────────┴──────────┴──────────┴──────────┘\n\n";

  double downsample_time = voxel_bench.avg();
  double ransac_time = ransac_bench.avg();
  double cluster_time = cluster_bench.avg();

  // ===========================================================================
  // Step 2: Downsampling (results from benchmark)
  // ===========================================================================
  std::cout << "━━━ Step 2: VoxelGrid Downsampling ━━━\n";
  float reduction = 100.0f * (1.0f - static_cast<float>(downsampled.size()) /
                                         static_cast<float>(input.size()));
  std::cout << "  Voxel size: " << voxel_size << " m\n";
  std::cout << "  Points:     " << input.size() << " -> " << downsampled.size()
            << " (" << std::setprecision(1) << reduction << "% reduction)\n";
  std::cout << "  Time:       " << std::setprecision(2) << downsample_time
            << " ms (avg)\n";

  // Save downsampled (light gray)
  PointCloud down_viz = downsampled;
  down_viz.enableColor();
  for (size_t i = 0; i < down_viz.size(); ++i) {
    down_viz.color()[i] = Color(200, 200, 200);
  }
  io::savePCD(output_dir + "/02_downsampled.pcd", down_viz);
  std::cout << "  Saved:      02_downsampled.pcd\n\n";

  // ===========================================================================
  // Step 3: Ground Segmentation (results from benchmark)
  // ===========================================================================
  std::cout << "━━━ Step 3: RANSAC Ground Removal ━━━\n";
  std::cout << "  Threshold:  0.15 m\n";
  std::cout << "  Ground:     " << ground_result.inliers.size() << " points\n";
  std::cout << "  Obstacles:  " << obstacle_indices.size() << " points\n";
  std::cout << "  Fitness:    " << std::setprecision(1)
            << (ground_result.fitness * 100) << "%\n";
  std::cout << "  Iterations: " << ground_result.iterations << "\n";
  std::cout << "  Time:       " << std::setprecision(2) << ransac_time
            << " ms (avg)\n";

  // Save ground (green)
  std::vector<size_t> ground_idx(ground_result.inliers.begin(),
                                 ground_result.inliers.end());
  PointCloud ground = downsampled[ground_idx];
  ground.enableColor();
  for (size_t i = 0; i < ground.size(); ++i) {
    ground.color()[i] = Color::Green();
  }
  io::savePCD(output_dir + "/03_ground.pcd", ground);
  std::cout << "  Saved:      03_ground.pcd (green)\n";

  // Save obstacles (white initially, will be colored by cluster)
  obstacles.enableColor();
  for (size_t i = 0; i < obstacles.size(); ++i) {
    obstacles.color()[i] = Color::White();
  }
  io::savePCD(output_dir + "/04_obstacles.pcd", obstacles);
  std::cout << "  Saved:      04_obstacles.pcd (white)\n\n";

  // ===========================================================================
  // Step 4: Euclidean Clustering (results from benchmark)
  // ===========================================================================
  std::cout << "━━━ Step 4: Euclidean Clustering ━━━\n";

  size_t noise_count = obstacles.size() - clusters.totalClusteredPoints();

  std::cout << "  Tolerance:  " << cluster_cfg.tolerance << " m\n";
  std::cout << "  Min/Max:    " << cluster_cfg.min_size << " / "
            << cluster_cfg.max_size << " points\n";
  std::cout << "  Clusters:   " << clusters.numClusters() << "\n";
  std::cout << "  Noise:      " << noise_count << " points\n";
  std::cout << "  Time:       " << std::setprecision(2) << cluster_time
            << " ms (avg)\n";

  // Apply colors to obstacles based on cluster ID
  segmentation::applyClusterLabels(obstacles, clusters);
  for (size_t i = 0; i < obstacles.size(); ++i) {
    uint16_t instance_id = obstacles.label()[i].instanceId();
    if (instance_id > 0) {
      obstacles.color()[i] = getClusterColor(instance_id - 1);
    } else {
      obstacles.color()[i] = Color(128, 128, 128); // Gray for noise
    }
  }
  io::savePCD(output_dir + "/05_clusters.pcd", obstacles);
  std::cout << "  Saved:      05_clusters.pcd (colored by cluster)\n\n";

  // ===========================================================================
  // Step 5: Bounding Box Analysis
  // ===========================================================================
  std::cout << "━━━ Step 5: Object Analysis ━━━\n";
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  "
               "┌────────┬────────┬───────────────────────────┬────────────────"
               "──────┐\n";
  std::cout << "  │ Cluster│ Points │      Center (x, y, z)     │   Size (W x "
               "L x H)   │\n";
  std::cout << "  "
               "├────────┼────────┼───────────────────────────┼────────────────"
               "──────┤\n";

  std::vector<BoundingBox> bboxes;
  for (size_t i = 0; i < clusters.numClusters(); ++i) {
    PointCloud cluster = clusters.extract(obstacles, i);
    BoundingBox bbox = computeBBox(cluster);
    bboxes.push_back(bbox);

    std::cout << "  │ " << std::setw(6) << i << " │ " << std::setw(6)
              << bbox.num_points << " │ (" << std::setw(6) << bbox.centroid.x()
              << ", " << std::setw(6) << bbox.centroid.y() << ", "
              << std::setw(5) << bbox.centroid.z() << ") │ " << std::setw(4)
              << bbox.width() << " x " << std::setw(4) << bbox.length() << " x "
              << std::setw(4) << bbox.height() << " │\n";
  }
  std::cout << "  "
               "└────────┴────────┴───────────────────────────┴────────────────"
               "──────┘\n\n";

  // ===========================================================================
  // Step 6: Merged Visualization
  // ===========================================================================
  std::cout << "━━━ Step 6: Final Merged Output ━━━\n";

  // Combine ground (green) + clustered obstacles (colored)
  PointCloud merged = ground + obstacles;
  io::savePCD(output_dir + "/06_final.pcd", merged);
  std::cout << "  Total:      " << merged.size() << " points\n";
  std::cout << "  Saved:      06_final.pcd (ground=green, objects=colored)\n\n";

  // ===========================================================================
  // Summary
  // ===========================================================================
  double pipeline_avg = downsample_time + ransac_time + cluster_time;
  double pipeline_min = total_bench.min_ms;

  std::cout
      << "╔════════════════════════════════════════════════════════════╗\n";
  std::cout
      << "║                      Pipeline Summary                      ║\n";
  std::cout
      << "╠════════════════════════════════════════════════════════════╣\n";
  std::cout << "║  Input points:        " << std::setw(10) << input.size()
            << " pts" << std::string(18, ' ') << "║\n";
  std::cout << "║  After downsampling:  " << std::setw(10) << downsampled.size()
            << " pts" << std::string(18, ' ') << "║\n";
  std::cout << "║  Ground points:       " << std::setw(10) << ground.size()
            << " pts" << std::string(18, ' ') << "║\n";
  std::cout << "║  Obstacle points:     " << std::setw(10) << obstacles.size()
            << " pts" << std::string(18, ' ') << "║\n";
  std::cout << "║  Detected objects:    " << std::setw(10)
            << clusters.numClusters() << std::string(22, ' ') << "║\n";
  std::cout
      << "╠════════════════════════════════════════════════════════════╣\n";
  std::cout << "║  Timing (avg of " << BENCHMARK_RUNS << " runs after "
            << WARMUP_RUNS << " warmup):              ║\n";
  std::cout << "║    - VoxelGrid:       " << std::setw(10) << downsample_time
            << " ms" << std::string(18, ' ') << "║\n";
  std::cout << "║    - RANSAC:          " << std::setw(10) << ransac_time
            << " ms" << std::string(18, ' ') << "║\n";
  std::cout << "║    - Clustering:      " << std::setw(10) << cluster_time
            << " ms" << std::string(18, ' ') << "║\n";
  std::cout
      << "║    ─────────────────────────────────                       ║\n";
  std::cout << "║    Pipeline (avg):    " << std::setw(10) << pipeline_avg
            << " ms" << std::string(18, ' ') << "║\n";
  std::cout << "║    Pipeline (min):    " << std::setw(10) << pipeline_min
            << " ms" << std::string(18, ' ') << "║\n";
  std::cout << "║    Throughput (min):  " << std::setw(10)
            << std::setprecision(1) << (1000.0 / pipeline_min) << " Hz"
            << std::string(18, ' ') << "║\n";
  std::cout
      << "╚════════════════════════════════════════════════════════════╝\n\n";

  std::cout << "Visualization:\n";
  std::cout << "  View all steps:    pcl_viewer " << output_dir << "/*.pcd\n";
  std::cout << "  View final result: pcl_viewer " << output_dir
            << "/06_final.pcd\n";
  std::cout << "  Step-by-step:      pcl_viewer " << output_dir
            << "/01_input.pcd " << output_dir << "/02_downsampled.pcd ...\n\n";

  return 0;
}
