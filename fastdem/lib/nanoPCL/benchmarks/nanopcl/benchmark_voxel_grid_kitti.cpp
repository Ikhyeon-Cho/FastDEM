#include <nanopcl/common.hpp>
#include <nanopcl/filters.hpp>

#ifdef HAVE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#endif

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

using Clock = std::chrono::high_resolution_clock;

// Load KITTI bin
nanopcl::PointCloud loadKittiBin(const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) throw std::runtime_error("Failed to open: " + path);
  file.seekg(0, std::ios::end);
  size_t num_points = file.tellg() / (4 * sizeof(float));
  file.seekg(0, std::ios::beg);

  nanopcl::PointCloud cloud;
  cloud.reserve(num_points);
  cloud.useIntensity(); // KITTI has intensity

  std::vector<float> buffer(4);
  for (size_t i = 0; i < num_points; ++i) {
    file.read(reinterpret_cast<char*>(buffer.data()), 4 * sizeof(float));
    cloud.add(buffer[0], buffer[1], buffer[2], nanopcl::Intensity{buffer[3]});
  }
  return cloud;
}

#ifdef HAVE_PCL
pcl::PointCloud<pcl::PointXYZI>::Ptr nanoToPCL(const nanopcl::PointCloud& cloud) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_cloud->resize(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    pcl_cloud->points[i].x = cloud.points()[i].x();
    pcl_cloud->points[i].y = cloud.points()[i].y();
    pcl_cloud->points[i].z = cloud.points()[i].z();
    pcl_cloud->points[i].intensity = cloud.intensities()[i];
  }
  return pcl_cloud;
}
#endif

template <typename F>
double measure(F&& func, int runs = 10) {
  func(); // Warmup
  auto start = Clock::now();
  for (int i = 0; i < runs; ++i) func();
  auto end = Clock::now();
  return std::chrono::duration<double, std::milli>(end - start).count() / runs;
}

int main(int argc, char** argv) {
  std::string data_path = (argc > 1) ? argv[1] : "../../data/kitti/000000.bin";
  
  std::cout << "Loading: " << data_path << " ... ";
  auto cloud = loadKittiBin(data_path);
  std::cout << cloud.size() << " points\n";

#ifdef HAVE_PCL
  auto pcl_cloud = nanoToPCL(cloud);
#endif

  std::vector<float> voxel_sizes = {0.1f, 0.5f};

  std::cout << "\n------------------------------------------------------------\n";
  std::cout << " VoxelGrid Benchmark (KITTI Data)\n";
  std::cout << "------------------------------------------------------------\n";
  std::cout << " Size  | PCL (ms) | nanoPCL (ms) | Speedup\n";
  std::cout << "-------|----------|--------------|--------\n";

  for (float voxel_size : voxel_sizes) {
    double pcl_time = 0.0;
    double nano_time = 0.0;

#ifdef HAVE_PCL
    {
      pcl::VoxelGrid<pcl::PointXYZI> grid;
      grid.setLeafSize(voxel_size, voxel_size, voxel_size);
      grid.setInputCloud(pcl_cloud);
      pcl::PointCloud<pcl::PointXYZI> out;
      pcl_time = measure([&]() { 
        grid.filter(out); 
      });
    }
#endif

    {
      nano_time = measure([&]() {
        auto out = nanopcl::filters::voxelGrid(cloud, voxel_size);
      });
    }

    std::cout << " " << std::fixed << std::setprecision(2) << voxel_size << "m  | "
              << std::setw(8) << pcl_time << " | "
              << std::setw(12) << nano_time << " | "
              << std::setprecision(1) << (pcl_time / nano_time) << "x\n";
  }
  std::cout << "------------------------------------------------------------\n";
  return 0;
}
