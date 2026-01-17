/**
 * @file benchmark_ros1_bridge.cpp
 * @brief Standalone benchmark for ROS1 bridge conversion
 *
 * Build:
 *   cd build && cmake .. -DNANOPCL_BUILD_BENCHMARKS=ON && make benchmark_ros1_bridge
 *
 * Usage:
 *   ./benchmark_ros1_bridge /path/to/bag /topic_name [num_messages]
 */

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

// nanoPCL bridge
#include <nanopcl/bridge/ros1.hpp>

// PCL (optional)
#ifdef HAS_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double, std::milli>;

// ============================================================================
// Simple baseline implementation (mimics typical ROS conversion code)
// ============================================================================
namespace baseline {

struct SimpleCloud {
  std::vector<Eigen::Vector3f> points;
  std::vector<float> intensity;
  std::string frame_id;
  uint64_t timestamp_ns;

  size_t size() const { return points.size(); }
  bool hasIntensity() const { return !intensity.empty(); }
};

SimpleCloud fromPointCloud2(const sensor_msgs::PointCloud2& msg) {
  SimpleCloud cloud;
  cloud.frame_id = msg.header.frame_id;
  cloud.timestamp_ns = msg.header.stamp.toNSec();

  const size_t num_points = msg.width * msg.height;
  cloud.points.reserve(num_points);

  // Find field offsets
  int x_off = -1, y_off = -1, z_off = -1, intensity_off = -1;
  for (const auto& field : msg.fields) {
    if (field.name == "x") x_off = field.offset;
    else if (field.name == "y") y_off = field.offset;
    else if (field.name == "z") z_off = field.offset;
    else if (field.name == "intensity") intensity_off = field.offset;
  }

  if (x_off < 0 || y_off < 0 || z_off < 0) return cloud;

  const bool has_intensity = (intensity_off >= 0);
  if (has_intensity) cloud.intensity.reserve(num_points);

  const uint8_t* data = msg.data.data();
  const size_t step = msg.point_step;

  for (size_t i = 0; i < num_points; ++i) {
    const uint8_t* ptr = data + i * step;
    float x = *reinterpret_cast<const float*>(ptr + x_off);
    float y = *reinterpret_cast<const float*>(ptr + y_off);
    float z = *reinterpret_cast<const float*>(ptr + z_off);

    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    cloud.points.emplace_back(x, y, z);

    if (has_intensity) {
      float intensity = *reinterpret_cast<const float*>(ptr + intensity_off);
      cloud.intensity.push_back(std::isfinite(intensity) ? intensity : 0.0f);
    }
  }

  return cloud;
}

sensor_msgs::PointCloud2 toPointCloud2(const SimpleCloud& cloud) {
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = cloud.frame_id;
  msg.header.stamp.fromNSec(cloud.timestamp_ns);
  msg.height = 1;
  msg.width = cloud.size();
  msg.is_bigendian = false;
  msg.is_dense = true;

  // Build fields
  size_t offset = 0;
  auto addField = [&](const std::string& name, uint8_t type) {
    sensor_msgs::PointField f;
    f.name = name;
    f.offset = offset;
    f.datatype = type;
    f.count = 1;
    msg.fields.push_back(f);
    offset += 4;
  };

  addField("x", sensor_msgs::PointField::FLOAT32);
  addField("y", sensor_msgs::PointField::FLOAT32);
  addField("z", sensor_msgs::PointField::FLOAT32);
  if (cloud.hasIntensity()) {
    addField("intensity", sensor_msgs::PointField::FLOAT32);
  }

  msg.point_step = offset;
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);

  uint8_t* data = msg.data.data();
  for (size_t i = 0; i < cloud.size(); ++i) {
    uint8_t* ptr = data + i * msg.point_step;
    *reinterpret_cast<float*>(ptr + 0) = cloud.points[i].x();
    *reinterpret_cast<float*>(ptr + 4) = cloud.points[i].y();
    *reinterpret_cast<float*>(ptr + 8) = cloud.points[i].z();
    if (cloud.hasIntensity()) {
      *reinterpret_cast<float*>(ptr + 12) = cloud.intensity[i];
    }
  }

  return msg;
}

}  // namespace baseline

// ============================================================================
// Benchmark utilities
// ============================================================================

struct BenchmarkResult {
  std::string name;
  std::vector<double> times_ms;
  size_t total_points = 0;

  double mean() const {
    if (times_ms.empty()) return 0;
    return std::accumulate(times_ms.begin(), times_ms.end(), 0.0) /
           times_ms.size();
  }

  double stddev() const {
    if (times_ms.size() < 2) return 0;
    double m = mean();
    double sq_sum = 0;
    for (double t : times_ms) sq_sum += (t - m) * (t - m);
    return std::sqrt(sq_sum / (times_ms.size() - 1));
  }

  double min() const {
    return times_ms.empty()
               ? 0
               : *std::min_element(times_ms.begin(), times_ms.end());
  }

  double max() const {
    return times_ms.empty()
               ? 0
               : *std::max_element(times_ms.begin(), times_ms.end());
  }

  double throughput() const {
    double total_time_sec = std::accumulate(times_ms.begin(), times_ms.end(), 0.0) / 1000.0;
    return total_time_sec > 0 ? total_points / total_time_sec : 0;
  }
};

void printResult(const BenchmarkResult& result) {
  std::cout << std::left << std::setw(20) << result.name << " | mean: "
            << std::fixed << std::setprecision(3) << std::setw(7)
            << result.mean() << " ms"
            << " | std: " << std::setw(5) << result.stddev() << " ms"
            << " | min: " << std::setw(5) << result.min() << " ms"
            << " | max: " << std::setw(6) << result.max() << " ms"
            << " | " << std::setprecision(0) << std::setw(8)
            << result.throughput() / 1e6 << " Mpts/s" << std::endl;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <bag_file> <topic> [num_messages]\n"
              << "Example: " << argv[0]
              << " ~/Downloads/campus_road_dataset.bag /velodyne/points 1000\n";
    return 1;
  }

  const std::string bag_file = argv[1];
  const std::string topic = argv[2];
  const size_t max_messages = (argc > 3) ? std::stoul(argv[3]) : 1000;

  std::cout << "=== ROS1 Bridge Benchmark ===\n"
            << "Bag: " << bag_file << "\n"
            << "Topic: " << topic << "\n"
            << "Max messages: " << max_messages << "\n\n";

  // Load messages
  std::vector<sensor_msgs::PointCloud2::ConstPtr> messages;
  {
    rosbag::Bag bag;
    try {
      bag.open(bag_file, rosbag::bagmode::Read);
    } catch (const rosbag::BagException& e) {
      std::cerr << "Failed to open bag: " << e.what() << std::endl;
      return 1;
    }

    rosbag::View view(bag, rosbag::TopicQuery(topic));
    for (const auto& m : view) {
      auto msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (msg) {
        messages.push_back(msg);
        if (messages.size() >= max_messages) break;
      }
    }
    bag.close();
  }

  if (messages.empty()) {
    std::cerr << "No messages found on topic: " << topic << std::endl;
    return 1;
  }

  // Option: strip to XYZI or use original
  const bool use_original = false;  // Set false for XYZI-only test (fair PCL comparison)

  auto stripToXYZI = [](const sensor_msgs::PointCloud2& in) {
    sensor_msgs::PointCloud2 out;
    out.header = in.header;
    out.height = 1;
    out.is_bigendian = false;
    out.is_dense = in.is_dense;

    // Find offsets in source
    int x_off = -1, y_off = -1, z_off = -1, i_off = -1;
    for (const auto& f : in.fields) {
      if (f.name == "x") x_off = f.offset;
      else if (f.name == "y") y_off = f.offset;
      else if (f.name == "z") z_off = f.offset;
      else if (f.name == "intensity") i_off = f.offset;
    }

    // Build output fields (XYZI only, 16 bytes)
    sensor_msgs::PointField fx, fy, fz, fi;
    fx.name = "x"; fx.offset = 0; fx.datatype = sensor_msgs::PointField::FLOAT32; fx.count = 1;
    fy.name = "y"; fy.offset = 4; fy.datatype = sensor_msgs::PointField::FLOAT32; fy.count = 1;
    fz.name = "z"; fz.offset = 8; fz.datatype = sensor_msgs::PointField::FLOAT32; fz.count = 1;
    fi.name = "intensity"; fi.offset = 12; fi.datatype = sensor_msgs::PointField::FLOAT32; fi.count = 1;
    out.fields = {fx, fy, fz, fi};
    out.point_step = 16;

    // Count valid points
    size_t num_points = in.width * in.height;
    const uint8_t* src = in.data.data();
    size_t valid_count = 0;

    for (size_t i = 0; i < num_points; ++i) {
      const uint8_t* p = src + i * in.point_step;
      float x = *reinterpret_cast<const float*>(p + x_off);
      float y = *reinterpret_cast<const float*>(p + y_off);
      float z = *reinterpret_cast<const float*>(p + z_off);
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) ++valid_count;
    }

    out.width = valid_count;
    out.row_step = out.point_step * out.width;
    out.data.resize(out.row_step);

    // Copy valid points
    uint8_t* dst = out.data.data();
    size_t dst_idx = 0;
    for (size_t i = 0; i < num_points; ++i) {
      const uint8_t* p = src + i * in.point_step;
      float x = *reinterpret_cast<const float*>(p + x_off);
      float y = *reinterpret_cast<const float*>(p + y_off);
      float z = *reinterpret_cast<const float*>(p + z_off);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

      uint8_t* d = dst + dst_idx * out.point_step;
      *reinterpret_cast<float*>(d + 0) = x;
      *reinterpret_cast<float*>(d + 4) = y;
      *reinterpret_cast<float*>(d + 8) = z;
      float intensity = (i_off >= 0) ? *reinterpret_cast<const float*>(p + i_off) : 0.0f;
      *reinterpret_cast<float*>(d + 12) = std::isfinite(intensity) ? intensity : 0.0f;
      ++dst_idx;
    }

    return out;
  };

  // Prepare test messages
  std::vector<sensor_msgs::PointCloud2::ConstPtr> test_messages;
  if (use_original) {
    test_messages = std::move(messages);
  } else {
    // Convert to XYZI-only format
    for (const auto& msg : messages) {
      test_messages.push_back(
          boost::make_shared<sensor_msgs::PointCloud2>(stripToXYZI(*msg)));
    }
    messages.clear();
  }

  // Print message info
  const auto& first = *test_messages[0];
  std::cout << "Loaded " << test_messages.size() << " messages"
            << (use_original ? " (original)" : " (stripped to XYZI)") << "\n"
            << "Points per message: " << first.width * first.height << "\n"
            << "Fields: ";
  for (const auto& f : first.fields) std::cout << f.name << " ";
  std::cout << "\n\n";

  // =========================================================================
  // Benchmark: PointCloud2 -> PointCloud
  // =========================================================================
  std::cout << "--- PointCloud2 -> PointCloud ---\n";

  BenchmarkResult baseline_from, nanopcl_from;
  baseline_from.name = "baseline";
  nanopcl_from.name = "nanoPCL bridge";

  // Warm-up
  for (size_t i = 0; i < std::min(size_t(10), test_messages.size()); ++i) {
    auto c1 = baseline::fromPointCloud2(*test_messages[i]);
    auto c2 = npcl::from(*test_messages[i]);
  }

  // Benchmark baseline
  for (const auto& msg : test_messages) {
    auto start = Clock::now();
    auto cloud = baseline::fromPointCloud2(*msg);
    auto end = Clock::now();
    baseline_from.times_ms.push_back(Duration(end - start).count());
    baseline_from.total_points += cloud.size();
  }

  // Benchmark nanoPCL
  for (const auto& msg : test_messages) {
    auto start = Clock::now();
    auto cloud = npcl::from(*msg);
    auto end = Clock::now();
    nanopcl_from.times_ms.push_back(Duration(end - start).count());
    nanopcl_from.total_points += cloud.size();
  }

  // Benchmark PCL (if available)
#ifdef HAS_PCL
  BenchmarkResult pcl_from;
  pcl_from.name = "PCL";
  for (const auto& msg : test_messages) {
    auto start = Clock::now();
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);
    auto end = Clock::now();
    pcl_from.times_ms.push_back(Duration(end - start).count());
    pcl_from.total_points += pcl_cloud.size();
  }
#endif

  printResult(baseline_from);
  printResult(nanopcl_from);
#ifdef HAS_PCL
  printResult(pcl_from);
#endif
  double speedup_from = baseline_from.mean() / nanopcl_from.mean();
  std::cout << "nanoPCL vs baseline: " << std::fixed << std::setprecision(2) << speedup_from << "x\n";
#ifdef HAS_PCL
  std::cout << "nanoPCL vs PCL: " << std::fixed << std::setprecision(2) << pcl_from.mean() / nanopcl_from.mean() << "x\n";
#endif
  std::cout << "\n";

  // =========================================================================
  // Benchmark: PointCloud -> PointCloud2
  // =========================================================================
  std::cout << "--- PointCloud -> PointCloud2 ---\n";

  // Prepare clouds
  std::vector<baseline::SimpleCloud> baseline_clouds;
  std::vector<npcl::PointCloud> nanopcl_clouds;
  baseline_clouds.reserve(test_messages.size());
  nanopcl_clouds.reserve(test_messages.size());

  for (const auto& msg : test_messages) {
    baseline_clouds.push_back(baseline::fromPointCloud2(*msg));
    nanopcl_clouds.push_back(npcl::from(*msg));
  }

  BenchmarkResult baseline_to, nanopcl_to;
  baseline_to.name = "baseline";
  nanopcl_to.name = "nanoPCL bridge";

  // Benchmark baseline
  for (const auto& cloud : baseline_clouds) {
    auto start = Clock::now();
    auto msg = baseline::toPointCloud2(cloud);
    auto end = Clock::now();
    baseline_to.times_ms.push_back(Duration(end - start).count());
    baseline_to.total_points += cloud.size();
  }

  // Benchmark nanoPCL
  for (const auto& cloud : nanopcl_clouds) {
    auto start = Clock::now();
    auto msg = npcl::to<sensor_msgs::PointCloud2>(cloud);
    auto end = Clock::now();
    nanopcl_to.times_ms.push_back(Duration(end - start).count());
    nanopcl_to.total_points += cloud.size();
  }

  // Benchmark PCL to() (if available)
#ifdef HAS_PCL
  BenchmarkResult pcl_to;
  pcl_to.name = "PCL";
  std::vector<pcl::PointCloud<pcl::PointXYZI>> pcl_clouds;
  pcl_clouds.reserve(test_messages.size());
  for (const auto& msg : test_messages) {
    pcl::PointCloud<pcl::PointXYZI> c;
    pcl::fromROSMsg(*msg, c);
    pcl_clouds.push_back(std::move(c));
  }
  for (const auto& pcl_cloud : pcl_clouds) {
    auto start = Clock::now();
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(pcl_cloud, msg);
    auto end = Clock::now();
    pcl_to.times_ms.push_back(Duration(end - start).count());
    pcl_to.total_points += pcl_cloud.size();
  }
#endif

  printResult(baseline_to);
  printResult(nanopcl_to);
#ifdef HAS_PCL
  printResult(pcl_to);
#endif
  double speedup_to = baseline_to.mean() / nanopcl_to.mean();
  std::cout << "nanoPCL vs baseline: " << std::fixed << std::setprecision(2) << speedup_to << "x\n";
#ifdef HAS_PCL
  std::cout << "nanoPCL vs PCL: " << std::fixed << std::setprecision(2) << pcl_to.mean() / nanopcl_to.mean() << "x\n";
#endif
  std::cout << "\n";

  // =========================================================================
  // Summary
  // =========================================================================
  std::cout << "=== Summary ===\n";
  double total_baseline = baseline_from.mean() + baseline_to.mean();
  double total_nanopcl = nanopcl_from.mean() + nanopcl_to.mean();
  std::cout << "Round-trip (from + to):\n"
            << "  Baseline:       " << std::fixed << std::setprecision(3)
            << total_baseline << " ms\n"
            << "  nanoPCL bridge: " << total_nanopcl << " ms\n"
            << "  Overall speedup: " << total_baseline / total_nanopcl
            << "x\n";

  return 0;
}
