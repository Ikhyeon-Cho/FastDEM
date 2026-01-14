// nanoPCL Benchmark: ROS Bridge Conversion Overhead
// Measures the cost of converting between ROS messages and nanoPCL types

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

// nanoPCL
#include <nanopcl/common.hpp>
#include <nanopcl/bridge/ros1.hpp>

// Timer utility
class Timer {
  std::chrono::high_resolution_clock::time_point start_;

 public:
  Timer() { start_ = std::chrono::high_resolution_clock::now(); }

  double elapsed_us() const {
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::micro>(end - start_).count();
  }

  double elapsed_ms() const { return elapsed_us() / 1000.0; }
};

// Prevent compiler optimization
template <typename T>
void doNotOptimize(T& value) {
  asm volatile("" : "+m"(value) : : "memory");
}

void printHeader(const std::string& title) {
  std::cout << "\n" << std::string(70, '=') << "\n";
  std::cout << title << "\n";
  std::cout << std::string(70, '=') << "\n";
}

void printSubHeader(const std::string& title) {
  std::cout << "\n--- " << title << " ---\n";
}

// Create a ROS PointCloud2 message with specified fields
sensor_msgs::PointCloud2 createRosMessage(size_t num_points, bool with_intensity,
                                          bool with_ring, bool with_time) {
  sensor_msgs::PointCloud2 msg;
  msg.header.frame_id = "base_link";
  msg.header.stamp = ros::Time(1234567890, 123456789);
  msg.height = 1;
  msg.width = num_points;
  msg.is_bigendian = false;
  msg.is_dense = true;

  // Build fields
  uint32_t offset = 0;
  auto addField = [&msg, &offset](const std::string& name, uint8_t datatype) {
    sensor_msgs::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = datatype;
    field.count = 1;
    msg.fields.push_back(field);
    offset += (datatype == sensor_msgs::PointField::FLOAT32)   ? 4
              : (datatype == sensor_msgs::PointField::UINT16)  ? 2
              : (datatype == sensor_msgs::PointField::UINT8)   ? 1
                                                               : 4;
  };

  addField("x", sensor_msgs::PointField::FLOAT32);
  addField("y", sensor_msgs::PointField::FLOAT32);
  addField("z", sensor_msgs::PointField::FLOAT32);
  if (with_intensity) addField("intensity", sensor_msgs::PointField::FLOAT32);
  if (with_ring) addField("ring", sensor_msgs::PointField::UINT16);
  if (with_time) addField("t", sensor_msgs::PointField::FLOAT32);

  msg.point_step = offset;
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);

  // Fill with random data
  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> intensity_dist(0.0f, 1.0f);
  std::uniform_int_distribution<uint16_t> ring_dist(0, 127);
  std::uniform_real_distribution<float> time_dist(0.0f, 0.1f);

  uint8_t* data_ptr = msg.data.data();
  for (size_t i = 0; i < num_points; ++i) {
    uint8_t* pt = data_ptr + i * msg.point_step;
    *reinterpret_cast<float*>(pt + 0) = pos_dist(gen);
    *reinterpret_cast<float*>(pt + 4) = pos_dist(gen);
    *reinterpret_cast<float*>(pt + 8) = pos_dist(gen);

    uint32_t off = 12;
    if (with_intensity) {
      *reinterpret_cast<float*>(pt + off) = intensity_dist(gen);
      off += 4;
    }
    if (with_ring) {
      *reinterpret_cast<uint16_t*>(pt + off) = ring_dist(gen);
      off += 2;
    }
    if (with_time) {
      *reinterpret_cast<float*>(pt + off) = time_dist(gen);
    }
  }

  return msg;
}

// Create nanoPCL PointCloud
npcl::PointCloud createNanoPclCloud(size_t num_points, bool with_intensity,
                                        bool with_ring, bool with_time) {
  npcl::PointCloud cloud(num_points);
  cloud.setFrameId("base_link");
  cloud.setTimestamp(1234567890123456789ULL);

  std::mt19937 gen(42);
  std::uniform_real_distribution<float> pos_dist(-50.0f, 50.0f);
  std::uniform_real_distribution<float> intensity_dist(0.0f, 1.0f);
  std::uniform_int_distribution<uint16_t> ring_dist(0, 127);
  std::uniform_real_distribution<float> time_dist(0.0f, 0.1f);

  if (with_intensity) cloud.enableIntensity();
  if (with_ring) cloud.enableRing();
  if (with_time) cloud.enableTime();

  for (size_t i = 0; i < num_points; ++i) {
    cloud.add(npcl::Point(pos_dist(gen), pos_dist(gen), pos_dist(gen)));
    if (with_intensity) cloud.intensity().back() = intensity_dist(gen);
    if (with_ring) cloud.ring().back() = ring_dist(gen);
    if (with_time) cloud.time().back() = time_dist(gen);
  }

  return cloud;
}

struct BenchmarkResult {
  double mean_us;
  double min_us;
  double max_us;
  double throughput_pts_per_us;
};

BenchmarkResult runBenchmark(std::function<void()> fn, int iterations,
                             size_t num_points) {
  std::vector<double> times;
  times.reserve(iterations);

  // Warmup
  for (int i = 0; i < 3; ++i) fn();

  // Measure
  for (int i = 0; i < iterations; ++i) {
    Timer t;
    fn();
    times.push_back(t.elapsed_us());
  }

  double sum = 0, min_t = times[0], max_t = times[0];
  for (double t : times) {
    sum += t;
    min_t = std::min(min_t, t);
    max_t = std::max(max_t, t);
  }

  BenchmarkResult result;
  result.mean_us = sum / iterations;
  result.min_us = min_t;
  result.max_us = max_t;
  result.throughput_pts_per_us = num_points / result.mean_us;
  return result;
}

void printResult(const std::string& name, const BenchmarkResult& r,
                 size_t num_points) {
  std::cout << std::setw(45) << std::left << name << std::fixed
            << std::setprecision(1) << std::setw(10) << std::right << r.mean_us
            << " us  (min: " << std::setw(8) << r.min_us
            << ", max: " << std::setw(8) << r.max_us << ")  "
            << std::setprecision(2) << r.throughput_pts_per_us
            << " M pts/sec\n";
}

int main(int argc, char** argv) {
  // Initialize ROS (needed for ros::Time)
  ros::Time::init();

  const std::vector<size_t> point_counts = {10000, 50000, 100000, 500000};
  const int NUM_ITERATIONS = 50;

  std::cout << "nanoPCL Bridge Benchmark: ROS Conversion Overhead\n";
  std::cout << "Iterations per test: " << NUM_ITERATIONS << "\n";

  // ============================================================================
  // 1. XYZ ONLY
  // ============================================================================
  printHeader("1. XYZ ONLY (12 bytes/point in ROS msg)");

  for (size_t num_points : point_counts) {
    printSubHeader(std::to_string(num_points) + " points");

    auto ros_msg = createRosMessage(num_points, false, false, false);
    auto nano_cloud = createNanoPclCloud(num_points, false, false, false);

    // ROS → nanoPCL
    npcl::PointCloud result_cloud;
    auto r1 = runBenchmark(
        [&]() {
          result_cloud = npcl::from(ros_msg);
          doNotOptimize(result_cloud[0].x());
        },
        NUM_ITERATIONS, num_points);
    printResult("ROS msg -> nanoPCL", r1, num_points);

    // nanoPCL → ROS
    sensor_msgs::PointCloud2 result_msg;
    auto r2 = runBenchmark(
        [&]() {
          result_msg = npcl::to<sensor_msgs::PointCloud2>(nano_cloud);
          doNotOptimize(result_msg.data[0]);
        },
        NUM_ITERATIONS, num_points);
    printResult("nanoPCL -> ROS msg", r2, num_points);

    // Roundtrip
    auto r3 = runBenchmark(
        [&]() {
          auto cloud = npcl::from(ros_msg);
          auto msg = npcl::to<sensor_msgs::PointCloud2>(cloud);
          doNotOptimize(msg.data[0]);
        },
        NUM_ITERATIONS, num_points);
    printResult("Roundtrip (ROS -> nano -> ROS)", r3, num_points);
  }

  // ============================================================================
  // 2. XYZ + INTENSITY
  // ============================================================================
  printHeader("2. XYZ + INTENSITY (16 bytes/point in ROS msg)");

  for (size_t num_points : point_counts) {
    printSubHeader(std::to_string(num_points) + " points");

    auto ros_msg = createRosMessage(num_points, true, false, false);
    auto nano_cloud = createNanoPclCloud(num_points, true, false, false);

    npcl::PointCloud result_cloud;
    auto r1 = runBenchmark(
        [&]() {
          result_cloud = npcl::from(ros_msg);
          doNotOptimize(result_cloud[0].x());
        },
        NUM_ITERATIONS, num_points);
    printResult("ROS msg -> nanoPCL", r1, num_points);

    sensor_msgs::PointCloud2 result_msg;
    auto r2 = runBenchmark(
        [&]() {
          result_msg = npcl::to<sensor_msgs::PointCloud2>(nano_cloud);
          doNotOptimize(result_msg.data[0]);
        },
        NUM_ITERATIONS, num_points);
    printResult("nanoPCL -> ROS msg", r2, num_points);
  }

  // ============================================================================
  // 3. XYZ + INTENSITY + RING + TIME (Full LiDAR)
  // ============================================================================
  printHeader("3. FULL LIDAR (XYZ + I + Ring + Time, ~22 bytes/point)");

  for (size_t num_points : point_counts) {
    printSubHeader(std::to_string(num_points) + " points");

    auto ros_msg = createRosMessage(num_points, true, true, true);
    auto nano_cloud = createNanoPclCloud(num_points, true, true, true);

    npcl::PointCloud result_cloud;
    auto r1 = runBenchmark(
        [&]() {
          result_cloud = npcl::from(ros_msg);
          doNotOptimize(result_cloud[0].x());
        },
        NUM_ITERATIONS, num_points);
    printResult("ROS msg -> nanoPCL", r1, num_points);

    sensor_msgs::PointCloud2 result_msg;
    auto r2 = runBenchmark(
        [&]() {
          result_msg = npcl::to<sensor_msgs::PointCloud2>(nano_cloud);
          doNotOptimize(result_msg.data[0]);
        },
        NUM_ITERATIONS, num_points);
    printResult("nanoPCL -> ROS msg", r2, num_points);
  }

  // ============================================================================
  // 4. TRANSFORM CONVERSION
  // ============================================================================
  printHeader("4. TRANSFORM CONVERSION");

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_link";
  tf_msg.header.stamp = ros::Time::now();
  tf_msg.transform.translation.x = 1.0;
  tf_msg.transform.translation.y = 2.0;
  tf_msg.transform.translation.z = 3.0;
  tf_msg.transform.rotation.w = 0.707;
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = 0.707;

  npcl::Transform nano_tf =
      npcl::Transform::fromRPY("map", "base_link", 0.0, 0.0, 1.57);

  const int TF_ITERATIONS = 100000;

  printSubHeader("Transform conversions x" + std::to_string(TF_ITERATIONS));

  // ROS → nanoPCL Transform
  {
    npcl::Transform result;
    Timer t;
    for (int i = 0; i < TF_ITERATIONS; ++i) {
      result = npcl::from(tf_msg);
      doNotOptimize(result);
    }
    double total_us = t.elapsed_us();
    std::cout << std::setw(45) << std::left << "ROS TransformStamped -> nanoPCL"
              << std::fixed << std::setprecision(3) << total_us / TF_ITERATIONS
              << " us/op  (" << TF_ITERATIONS / (total_us / 1e6)
              << " ops/sec)\n";
  }

  // nanoPCL → ROS Transform
  {
    geometry_msgs::TransformStamped result;
    Timer t;
    for (int i = 0; i < TF_ITERATIONS; ++i) {
      result = npcl::toTransformStampedMsg(nano_tf);
      doNotOptimize(result);
    }
    double total_us = t.elapsed_us();
    std::cout << std::setw(45) << std::left << "nanoPCL -> ROS TransformStamped"
              << std::fixed << std::setprecision(3) << total_us / TF_ITERATIONS
              << " us/op  (" << TF_ITERATIONS / (total_us / 1e6)
              << " ops/sec)\n";
  }

  // ============================================================================
  // 5. COMPARISON WITH PROCESSING COST
  // ============================================================================
  printHeader("5. CONVERSION vs PROCESSING COST COMPARISON");

  const size_t comparison_points = 100000;
  auto ros_msg = createRosMessage(comparison_points, true, false, false);
  auto nano_cloud = createNanoPclCloud(comparison_points, true, false, false);

  printSubHeader("100k points - Conversion vs Voxel Filter vs Transform");

  // Conversion cost
  auto conv_result = runBenchmark(
      [&]() {
        auto cloud = npcl::from(ros_msg);
        doNotOptimize(cloud[0].x());
      },
      NUM_ITERATIONS, comparison_points);
  printResult("ROS -> nanoPCL conversion", conv_result, comparison_points);

  // Voxel filter cost
  auto voxel_result = runBenchmark(
      [&]() {
        auto filtered =
            npcl::filters::voxelGrid(nano_cloud, 0.1f);
        doNotOptimize(filtered[0].x());
      },
      NUM_ITERATIONS, comparison_points);
  printResult("Voxel filter (0.1m)", voxel_result, comparison_points);

  // Transform cost
  Eigen::Isometry3f tf = Eigen::Isometry3f::Identity();
  tf.rotate(Eigen::AngleAxisf(0.1f, Eigen::Vector3f::UnitZ()));
  tf.pretranslate(Eigen::Vector3f(1.0f, 2.0f, 3.0f));

  auto transform_result = runBenchmark(
      [&]() {
        auto transformed = npcl::transformCloud(nano_cloud, tf);
        doNotOptimize(transformed[0].x());
      },
      NUM_ITERATIONS, comparison_points);
  printResult("Cloud transform", transform_result, comparison_points);

  // Full pipeline: convert + filter + transform + convert back
  auto pipeline_result = runBenchmark(
      [&]() {
        auto cloud = npcl::from(ros_msg);
        cloud = npcl::filters::voxelGrid(std::move(cloud), 0.1f);
        cloud = npcl::transformCloud(std::move(cloud), tf);
        auto out_msg = npcl::to<sensor_msgs::PointCloud2>(cloud);
        doNotOptimize(out_msg.data[0]);
      },
      NUM_ITERATIONS, comparison_points);
  printResult("Full pipeline (conv+filter+tf+conv)", pipeline_result,
              comparison_points);

  // ============================================================================
  // SUMMARY
  // ============================================================================
  printHeader("SUMMARY");
  std::cout << R"(
Conversion overhead analysis:
  - ROS msg -> nanoPCL: ~X us per 100k points
  - nanoPCL -> ROS msg: ~Y us per 100k points
  - Transform conversion: <1 us per operation

Recommendations:
  - Conversion overhead is typically small compared to processing
  - Use move semantics in pipelines to minimize copies
  - For tight loops, keep data in nanoPCL format
  - Convert at ROS boundaries (subscriber/publisher)

Note: Actual performance depends on:
  - Number and types of channels
  - Memory layout (is_dense, field alignment)
  - CPU cache effects
)";

  return 0;
}
