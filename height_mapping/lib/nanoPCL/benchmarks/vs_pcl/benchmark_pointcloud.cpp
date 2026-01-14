// nanoPCL vs PCL Benchmark: PointCloud Data Structure Performance
// Compares SoA (nanoPCL) vs AoS (PCL) memory layout and access patterns

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

// nanoPCL
#include <nanopcl/common.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Timer utility
class Timer {
  std::chrono::high_resolution_clock::time_point start_;
  std::string name_;

 public:
  Timer(const std::string& name) : name_(name) {
    start_ = std::chrono::high_resolution_clock::now();
  }
  double elapsed_ms() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }
  ~Timer() {
    std::cout << std::setw(40) << std::left << name_ << ": " << std::fixed
              << std::setprecision(3) << elapsed_ms() << " ms\n";
  }
};

// Prevent compiler optimization
template <typename T>
void doNotOptimize(T& value) {
  asm volatile("" : "+r"(value) : : "memory");
}

void printHeader(const std::string& title) {
  std::cout << "\n" << std::string(60, '=') << "\n";
  std::cout << title << "\n";
  std::cout << std::string(60, '=') << "\n";
}

void printSubHeader(const std::string& title) {
  std::cout << "\n--- " << title << " ---\n";
}

int main() {
  const int NUM_POINTS = 500000;  // 500k points (typical LiDAR scan)
  const int NUM_ITERATIONS = 100;

  std::cout << "nanoPCL vs PCL Benchmark: PointCloud Data Structure\n";
  std::cout << "Points: " << NUM_POINTS << ", Iterations: " << NUM_ITERATIONS
            << "\n";

  // Random data generator
  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> intensity_dist(0.0f, 1.0f);
  std::uniform_int_distribution<uint16_t> ring_dist(0, 127);

  // ============================================================================
  // 1. CONSTRUCTION & MEMORY
  // ============================================================================
  printHeader("1. CONSTRUCTION & MEMORY");

  npcl::PointCloud nano_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_xyzi(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcl_cloud_full(
      new pcl::PointCloud<pcl::PointXYZINormal>);

  printSubHeader("Construction with reserve");
  {
    Timer t("[nanoPCL] reserve + enableIntensity");
    npcl::PointCloud cloud;
    cloud.reserve(NUM_POINTS);
    cloud.enableIntensity();
  }
  {
    Timer t("[PCL XYZI] resize");
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.points.resize(NUM_POINTS);
  }
  {
    Timer t("[PCL XYZINormal] resize");
    pcl::PointCloud<pcl::PointXYZINormal> cloud;
    cloud.points.resize(NUM_POINTS);
  }

  // Memory comparison
  printSubHeader("Memory per point (bytes)");
  std::cout << "[nanoPCL] Point only: " << sizeof(npcl::Point) << "\n";
  std::cout << "[nanoPCL] + intensity: "
            << sizeof(npcl::Point) + sizeof(float) << "\n";
  std::cout << "[nanoPCL] + intensity + ring: "
            << sizeof(npcl::Point) + sizeof(float) + sizeof(uint16_t)
            << "\n";
  std::cout << "[PCL] PointXYZ: " << sizeof(pcl::PointXYZ) << "\n";
  std::cout << "[PCL] PointXYZI: " << sizeof(pcl::PointXYZI) << "\n";
  std::cout << "[PCL] PointXYZINormal: " << sizeof(pcl::PointXYZINormal)
            << "\n";

  // ============================================================================
  // 2. POPULATE DATA
  // ============================================================================
  printHeader("2. POPULATE DATA");

  printSubHeader("push_back " + std::to_string(NUM_POINTS) + " points");
  {
    Timer t("[nanoPCL] push_back with Intensity");
    nano_cloud.reserve(NUM_POINTS);
    nano_cloud.enableIntensity();
    for (int i = 0; i < NUM_POINTS; ++i) {
      nano_cloud.add(
          npcl::Point(pos_dist(gen), pos_dist(gen), pos_dist(gen)),
          npcl::Intensity(intensity_dist(gen)));
    }
  }

  gen.seed(42);  // Reset seed for same data
  {
    Timer t("[PCL XYZI] direct assignment");
    pcl_cloud_xyzi->points.resize(NUM_POINTS);
    pcl_cloud_xyzi->width = NUM_POINTS;
    pcl_cloud_xyzi->height = 1;
    for (int i = 0; i < NUM_POINTS; ++i) {
      pcl_cloud_xyzi->points[i].x = pos_dist(gen);
      pcl_cloud_xyzi->points[i].y = pos_dist(gen);
      pcl_cloud_xyzi->points[i].z = pos_dist(gen);
      pcl_cloud_xyzi->points[i].intensity = intensity_dist(gen);
    }
  }

  // ============================================================================
  // 3. SEQUENTIAL ACCESS (Cache-friendly)
  // ============================================================================
  printHeader("3. SEQUENTIAL ACCESS (XYZ only)");

  printSubHeader("Sum all Z coordinates x" + std::to_string(NUM_ITERATIONS));
  float sum = 0;
  {
    Timer t("[nanoPCL] range-based for");
    for (int iter = 0; iter < NUM_ITERATIONS; ++iter) {
      sum = 0;
      for (const auto& p : nano_cloud) {
        sum += p.z();
      }
      doNotOptimize(sum);
    }
  }
  std::cout << "  Result: " << sum << "\n";

  {
    Timer t("[PCL XYZI] range-based for");
    for (int iter = 0; iter < NUM_ITERATIONS; ++iter) {
      sum = 0;
      for (const auto& p : pcl_cloud_xyzi->points) {
        sum += p.z;
      }
      doNotOptimize(sum);
    }
  }
  std::cout << "  Result: " << sum << "\n";

  // ============================================================================
  // 4. SEQUENTIAL ACCESS (XYZ + Intensity)
  // ============================================================================
  printHeader("4. SEQUENTIAL ACCESS (XYZ + Intensity)");

  printSubHeader("Weighted sum (z * intensity) x" +
                 std::to_string(NUM_ITERATIONS));
  {
    Timer t("[nanoPCL] separate arrays");
    for (int iter = 0; iter < NUM_ITERATIONS; ++iter) {
      sum = 0;
      const auto& pts = nano_cloud.xyz();
      const auto& intensities = nano_cloud.intensity();
      for (size_t i = 0; i < nano_cloud.size(); ++i) {
        sum += pts[i].z() * intensities[i];
      }
      doNotOptimize(sum);
    }
  }
  std::cout << "  Result: " << sum << "\n";

  {
    Timer t("[PCL XYZI] interleaved");
    for (int iter = 0; iter < NUM_ITERATIONS; ++iter) {
      sum = 0;
      for (const auto& p : pcl_cloud_xyzi->points) {
        sum += p.z * p.intensity;
      }
      doNotOptimize(sum);
    }
  }
  std::cout << "  Result: " << sum << "\n";

  // ============================================================================
  // 5. RANDOM ACCESS
  // ============================================================================
  printHeader("5. RANDOM ACCESS");

  // Generate random indices
  std::vector<size_t> random_indices(NUM_POINTS);
  for (int i = 0; i < NUM_POINTS; ++i) random_indices[i] = i;
  std::shuffle(random_indices.begin(), random_indices.end(), gen);

  printSubHeader("Random index access x" + std::to_string(NUM_ITERATIONS / 10));
  {
    Timer t("[nanoPCL] random access");
    for (int iter = 0; iter < NUM_ITERATIONS / 10; ++iter) {
      sum = 0;
      for (size_t idx : random_indices) {
        sum += nano_cloud[idx].z();
      }
      doNotOptimize(sum);
    }
  }

  {
    Timer t("[PCL XYZI] random access");
    for (int iter = 0; iter < NUM_ITERATIONS / 10; ++iter) {
      sum = 0;
      for (size_t idx : random_indices) {
        sum += pcl_cloud_xyzi->points[idx].z;
      }
      doNotOptimize(sum);
    }
  }

  // ============================================================================
  // 6. TRANSFORM OPERATION
  // ============================================================================
  printHeader("6. TRANSFORM OPERATION");

  Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();
  tf.rotate(Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitZ()));
  tf.pretranslate(Eigen::Vector3f(1.0f, 2.0f, 3.0f));

  printSubHeader("Apply 4x4 transform x" + std::to_string(NUM_ITERATIONS / 10));
  {
    Timer t("[nanoPCL] transform (contiguous xyz)");
    for (int iter = 0; iter < NUM_ITERATIONS / 10; ++iter) {
      npcl::PointCloud copy = npcl::transformCloud(nano_cloud, tf);
      doNotOptimize(copy[0].x());
    }
  }

  {
    Timer timer("[PCL XYZI] manual transform");
    Eigen::Matrix3f R = tf.rotation();
    Eigen::Vector3f t = tf.translation();
    for (int iter = 0; iter < NUM_ITERATIONS / 10; ++iter) {
      pcl::PointCloud<pcl::PointXYZI> copy = *pcl_cloud_xyzi;
      for (auto& p : copy.points) {
        Eigen::Vector3f pt(p.x, p.y, p.z);
        pt = R * pt + t;
        p.x = pt.x();
        p.y = pt.y();
        p.z = pt.z();
      }
      doNotOptimize(copy.points[0].x);
    }
  }

  // ============================================================================
  // 7. COPY OPERATION
  // ============================================================================
  printHeader("7. COPY OPERATION");

  printSubHeader("Deep copy x" + std::to_string(NUM_ITERATIONS / 10));
  {
    Timer t("[nanoPCL] copy constructor");
    for (int iter = 0; iter < NUM_ITERATIONS / 10; ++iter) {
      npcl::PointCloud copy = nano_cloud;
      doNotOptimize(copy[0].x());
    }
  }

  {
    Timer t("[PCL XYZI] copy constructor");
    for (int iter = 0; iter < NUM_ITERATIONS / 10; ++iter) {
      pcl::PointCloud<pcl::PointXYZI> copy = *pcl_cloud_xyzi;
      doNotOptimize(copy.points[0].x);
    }
  }

  // ============================================================================
  // 8. OPTIONAL CHANNELS (nanoPCL advantage)
  // ============================================================================
  printHeader("8. OPTIONAL CHANNELS (nanoPCL-specific)");

  printSubHeader("XYZ-only iteration (no intensity overhead)");
  {
    npcl::PointCloud xyz_only;
    xyz_only.reserve(NUM_POINTS);
    gen.seed(42);
    for (int i = 0; i < NUM_POINTS; ++i) {
      xyz_only.add(
          npcl::Point(pos_dist(gen), pos_dist(gen), pos_dist(gen)));
    }

    Timer t("[nanoPCL] XYZ-only cloud");
    for (int iter = 0; iter < NUM_ITERATIONS; ++iter) {
      sum = 0;
      for (const auto& p : xyz_only) {
        sum += p.z();
      }
      doNotOptimize(sum);
    }
  }
  std::cout << "  (PCL always allocates intensity even if unused)\n";

  // ============================================================================
  // SUMMARY
  // ============================================================================
  printHeader("SUMMARY");
  std::cout << R"(
nanoPCL (SoA) advantages:
  - Smaller memory when not all attributes needed
  - Better cache utilization for XYZ-only operations
  - Optional channels reduce memory footprint

PCL (AoS) advantages:
  - Single allocation for all point data
  - Better when accessing all attributes together
  - More mature ecosystem

Recommendation:
  - Use nanoPCL for LiDAR processing with selective attributes
  - Use PCL for full-featured point cloud processing
)";

  return 0;
}
