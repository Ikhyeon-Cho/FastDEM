// nanoPCL vs PCL Benchmark: Voxel Grid Accuracy
// Compares voxel downsampling results between nanoPCL and PCL

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

// nanoPCL
#include <nanopcl/common.hpp>

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void printHeader(const std::string& title) {
  std::cout << "\n" << std::string(70, '=') << "\n";
  std::cout << title << "\n";
  std::cout << std::string(70, '=') << "\n";
}

void printSubHeader(const std::string& title) {
  std::cout << "\n--- " << title << " ---\n";
}

// Create identical point clouds for both libraries
void createTestClouds(size_t num_points, unsigned int seed,
                      nanopcl::PointCloud& nano_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_cloud) {
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> pos(-50.0f, 50.0f);
  std::uniform_real_distribution<float> intensity(0.0f, 1.0f);

  // nanoPCL
  nano_cloud = nanopcl::PointCloud();
  nano_cloud.useIntensity();
  nano_cloud.reserve(num_points);

  // PCL
  pcl_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl_cloud->reserve(num_points);

  for (size_t i = 0; i < num_points; ++i) {
    float x = pos(gen);
    float y = pos(gen);
    float z = pos(gen);
    float I = intensity(gen);

    nano_cloud.add(x, y, z, nanopcl::Intensity(I));

    pcl::PointXYZI pt;
    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.intensity = I;
    pcl_cloud->push_back(pt);
  }
}

// Create ground plane test (sensitive to z=0 boundary)
void createGroundPlaneTestClouds(nanopcl::PointCloud& nano_cloud,
                                  pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_cloud) {
  nano_cloud = nanopcl::PointCloud();
  nano_cloud.useIntensity();

  pcl_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> noise(-0.01f, 0.01f);

  // Ground plane at z ≈ 0 with small noise
  for (float x = -10.0f; x <= 10.0f; x += 0.2f) {
    for (float y = -10.0f; y <= 10.0f; y += 0.2f) {
      float z = noise(gen);  // Small noise around z=0

      nano_cloud.add(x, y, z, nanopcl::Intensity(0.5f));

      pcl::PointXYZI pt;
      pt.x = x;
      pt.y = y;
      pt.z = z;
      pt.intensity = 0.5f;
      pcl_cloud->push_back(pt);
    }
  }
}

// Compare two point clouds
struct ComparisonResult {
  size_t nano_count;
  size_t pcl_count;
  float min_z_nano;
  float max_z_nano;
  float min_z_pcl;
  float max_z_pcl;
  size_t matched_voxels;  // Points in same voxel location
  float avg_position_diff;
};

ComparisonResult compareResults(const nanopcl::PointCloud& nano,
                                 const pcl::PointCloud<pcl::PointXYZI>& pcl,
                                 float voxel_size) {
  ComparisonResult result{};
  result.nano_count = nano.size();
  result.pcl_count = pcl.size();

  if (nano.empty() || pcl.empty()) return result;

  // Find Z range for nanoPCL
  result.min_z_nano = nano[0].z();
  result.max_z_nano = nano[0].z();
  for (size_t i = 0; i < nano.size(); ++i) {
    result.min_z_nano = std::min(result.min_z_nano, nano[i].z());
    result.max_z_nano = std::max(result.max_z_nano, nano[i].z());
  }

  // Find Z range for PCL
  result.min_z_pcl = pcl[0].z;
  result.max_z_pcl = pcl[0].z;
  for (const auto& pt : pcl) {
    result.min_z_pcl = std::min(result.min_z_pcl, pt.z);
    result.max_z_pcl = std::max(result.max_z_pcl, pt.z);
  }

  // Build voxel map from PCL results
  auto toVoxelKey = [voxel_size](float x, float y, float z) {
    int ix = static_cast<int>(std::floor(x / voxel_size));
    int iy = static_cast<int>(std::floor(y / voxel_size));
    int iz = static_cast<int>(std::floor(z / voxel_size));
    return std::make_tuple(ix, iy, iz);
  };

  std::map<std::tuple<int, int, int>, Eigen::Vector3f> pcl_voxels;
  for (const auto& pt : pcl) {
    auto key = toVoxelKey(pt.x, pt.y, pt.z);
    pcl_voxels[key] = Eigen::Vector3f(pt.x, pt.y, pt.z);
  }

  // Compare nanoPCL points against PCL voxels
  float total_diff = 0;
  size_t matched = 0;
  for (size_t i = 0; i < nano.size(); ++i) {
    auto key = toVoxelKey(nano[i].x(), nano[i].y(), nano[i].z());
    auto it = pcl_voxels.find(key);
    if (it != pcl_voxels.end()) {
      ++matched;
      Eigen::Vector3f nano_pt(nano[i].x(), nano[i].y(), nano[i].z());
      total_diff += (nano_pt - it->second).norm();
    }
  }

  result.matched_voxels = matched;
  result.avg_position_diff = matched > 0 ? total_diff / matched : 0;

  return result;
}

void printComparison(const std::string& test_name,
                     const ComparisonResult& result,
                     float voxel_size) {
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "\n" << test_name << ":\n";
  std::cout << "  Point count:     nanoPCL=" << result.nano_count
            << ", PCL=" << result.pcl_count;
  if (result.nano_count == result.pcl_count) {
    std::cout << " ✓ MATCH\n";
  } else {
    std::cout << " ✗ DIFFER (diff=" << (int)result.nano_count - (int)result.pcl_count << ")\n";
  }

  std::cout << "  Z range nanoPCL: [" << result.min_z_nano << ", " << result.max_z_nano << "]\n";
  std::cout << "  Z range PCL:     [" << result.min_z_pcl << ", " << result.max_z_pcl << "]\n";

  float z_range_nano = result.max_z_nano - result.min_z_nano;
  float z_range_pcl = result.max_z_pcl - result.min_z_pcl;
  if (std::abs(z_range_nano - z_range_pcl) < voxel_size) {
    std::cout << "  Z range diff:    " << std::abs(z_range_nano - z_range_pcl) << " ✓ OK\n";
  } else {
    std::cout << "  Z range diff:    " << std::abs(z_range_nano - z_range_pcl) << " ✗ LARGE\n";
  }

  std::cout << "  Matched voxels:  " << result.matched_voxels << "/" << result.nano_count;
  float match_rate = 100.0f * result.matched_voxels / std::max<size_t>(1, result.nano_count);
  if (match_rate > 95) {
    std::cout << " (" << match_rate << "%) ✓\n";
  } else {
    std::cout << " (" << match_rate << "%) ✗\n";
  }

  std::cout << "  Avg position diff: " << result.avg_position_diff << "m";
  if (result.avg_position_diff < voxel_size * 0.1f) {
    std::cout << " ✓ (< 10% voxel size)\n";
  } else {
    std::cout << " ✗ (> 10% voxel size)\n";
  }
}

int main() {
  std::cout << "nanoPCL vs PCL Voxel Grid Accuracy Benchmark\n";
  std::cout << std::string(70, '=') << "\n";
  std::cout << "\nNote: PCL VoxelGrid uses centroid mode (average of points)\n";
  std::cout << "      nanoPCL CENTROID mode should match PCL results\n";

  // =========================================================================
  // TEST 1: Random point cloud
  // =========================================================================
  printHeader("TEST 1: Random Point Cloud (100k points, voxel=0.5m)");

  {
    const float VOXEL_SIZE = 0.5f;
    nanopcl::PointCloud nano_input;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_input;
    createTestClouds(100000, 42, nano_input, pcl_input);

    std::cout << "Input: " << nano_input.size() << " points\n";

    // PCL VoxelGrid
    auto pcl_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(pcl_input);
    vg.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    vg.filter(*pcl_filtered);

    // nanoPCL CENTROID
    auto nano_centroid = nanopcl::filters::voxelGrid(
        nano_input, VOXEL_SIZE, nanopcl::filters::VoxelMode::CENTROID);

    // nanoPCL CENTER
    auto nano_center = nanopcl::filters::voxelGrid(
        nano_input, VOXEL_SIZE, nanopcl::filters::VoxelMode::CENTER);

    auto result_centroid = compareResults(nano_centroid, *pcl_filtered, VOXEL_SIZE);
    auto result_center = compareResults(nano_center, *pcl_filtered, VOXEL_SIZE);

    printComparison("nanoPCL CENTROID vs PCL", result_centroid, VOXEL_SIZE);
    printComparison("nanoPCL CENTER vs PCL", result_center, VOXEL_SIZE);
  }

  // =========================================================================
  // TEST 2: Ground plane (z ≈ 0 boundary case)
  // =========================================================================
  printHeader("TEST 2: Ground Plane (z≈0, voxel=0.4m) - Boundary Case");

  {
    const float VOXEL_SIZE = 0.4f;
    nanopcl::PointCloud nano_input;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_input;
    createGroundPlaneTestClouds(nano_input, pcl_input);

    std::cout << "Input: " << nano_input.size() << " points (ground plane with z noise ±0.01)\n";

    // PCL VoxelGrid
    auto pcl_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(pcl_input);
    vg.setLeafSize(VOXEL_SIZE, VOXEL_SIZE, VOXEL_SIZE);
    vg.filter(*pcl_filtered);

    // nanoPCL CENTROID
    auto nano_centroid = nanopcl::filters::voxelGrid(
        nano_input, VOXEL_SIZE, nanopcl::filters::VoxelMode::CENTROID);

    // nanoPCL CENTER
    auto nano_center = nanopcl::filters::voxelGrid(
        nano_input, VOXEL_SIZE, nanopcl::filters::VoxelMode::CENTER);

    auto result_centroid = compareResults(nano_centroid, *pcl_filtered, VOXEL_SIZE);
    auto result_center = compareResults(nano_center, *pcl_filtered, VOXEL_SIZE);

    printComparison("nanoPCL CENTROID vs PCL", result_centroid, VOXEL_SIZE);
    printComparison("nanoPCL CENTER vs PCL", result_center, VOXEL_SIZE);

    printSubHeader("Z Distribution Analysis");

    // Count z layers for CENTER mode
    std::map<int, int> z_layers_center;
    for (size_t i = 0; i < nano_center.size(); ++i) {
      int iz = static_cast<int>(std::round(nano_center[i].z() / VOXEL_SIZE * 2));
      z_layers_center[iz]++;
    }
    std::cout << "nanoPCL CENTER z-layers:\n";
    for (auto& [iz, count] : z_layers_center) {
      float z = iz * VOXEL_SIZE / 2;
      std::cout << "  z ≈ " << std::setw(6) << z << "m: " << count << " points\n";
    }

    // Count z layers for PCL
    std::map<int, int> z_layers_pcl;
    for (const auto& pt : *pcl_filtered) {
      int iz = static_cast<int>(std::round(pt.z / VOXEL_SIZE * 2));
      z_layers_pcl[iz]++;
    }
    std::cout << "PCL z-layers:\n";
    for (auto& [iz, count] : z_layers_pcl) {
      float z = iz * VOXEL_SIZE / 2;
      std::cout << "  z ≈ " << std::setw(6) << z << "m: " << count << " points\n";
    }
  }

  // =========================================================================
  // TEST 3: Different voxel sizes
  // =========================================================================
  printHeader("TEST 3: Voxel Size Comparison");

  {
    nanopcl::PointCloud nano_input;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_input;
    createTestClouds(50000, 123, nano_input, pcl_input);

    std::cout << "Input: " << nano_input.size() << " points\n";
    std::cout << std::left << std::setw(12) << "Voxel Size"
              << std::setw(15) << "PCL Count"
              << std::setw(15) << "nanoPCL Count"
              << std::setw(10) << "Match"
              << "\n";
    std::cout << std::string(52, '-') << "\n";

    for (float voxel_size : {0.1f, 0.25f, 0.5f, 1.0f, 2.0f}) {
      // PCL
      auto pcl_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(pcl_input);
      vg.setLeafSize(voxel_size, voxel_size, voxel_size);
      vg.filter(*pcl_filtered);

      // nanoPCL
      auto nano_filtered = nanopcl::filters::voxelGrid(
          nano_input, voxel_size, nanopcl::filters::VoxelMode::CENTROID);

      std::cout << std::fixed << std::setprecision(2);
      std::cout << std::left << std::setw(12) << voxel_size
                << std::setw(15) << pcl_filtered->size()
                << std::setw(15) << nano_filtered.size();

      if (pcl_filtered->size() == nano_filtered.size()) {
        std::cout << "✓\n";
      } else {
        int diff = static_cast<int>(nano_filtered.size()) - static_cast<int>(pcl_filtered->size());
        std::cout << "✗ (" << (diff > 0 ? "+" : "") << diff << ")\n";
      }
    }
  }

  // =========================================================================
  // SUMMARY
  // =========================================================================
  printHeader("SUMMARY");
  std::cout << R"(
Key findings:
1. nanoPCL CENTROID mode should produce same point COUNT as PCL VoxelGrid
2. nanoPCL CENTER mode may have DIFFERENT z-distribution due to voxel center snapping
3. Small differences in point positions are expected (floating point, algorithm variations)

If point counts differ significantly:
- Check voxel boundary handling (floor vs round)
- Check coordinate range limits
- Compare voxel key calculation methods
)";

  return 0;
}
