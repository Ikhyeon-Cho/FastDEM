#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/TransformStamped.h>

class PointCloudOps {
public:
  template <typename T> using Cloud = pcl::PointCloud<T>;
  template <typename T> using CloudPtr = typename Cloud<T>::Ptr;

  static Eigen::Affine3d toAffine3d(const geometry_msgs::Transform &t) {
    Eigen::Translation3d translation(t.translation.x, t.translation.y, t.translation.z);
    Eigen::Quaterniond rotation(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
    return translation * rotation;
  }

  template <typename T>
  static CloudPtr<T> applyTransform(const CloudPtr<T> &input,
                                    const geometry_msgs::TransformStamped &ts) {
    if (input->header.frame_id.empty() || input->empty())
      return input;
    if (ts.child_frame_id == ts.header.frame_id)
      return input;

    auto affine = toAffine3d(ts.transform);
    auto output = boost::make_shared<Cloud<T>>();
    pcl::transformPointCloud(*input, *output, affine);
    output->header = input->header;
    output->header.frame_id = ts.header.frame_id;
    return output;
  }

  template <typename T>
  static CloudPtr<T> passThrough(const CloudPtr<T> &input,
                                 const std::string &field,
                                 double minVal,
                                 double maxVal,
                                 bool invert = false) {
    if (input->empty())
      return input;
    auto output = boost::make_shared<Cloud<T>>();
    pcl::PassThrough<T> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName(field);
    pass.setFilterLimits(minVal, maxVal);
    pass.setFilterLimitsNegative(invert);
    pass.filter(*output);
    output->header = input->header;
    return output;
  }

  template <typename T>
  static CloudPtr<T>
  filterRange2D(const CloudPtr<T> &input, double minRange, double maxRange) {
    if (input->empty())
      return input;
    auto output = boost::make_shared<Cloud<T>>();
    output->header = input->header;
    for (const auto &pt : input->points) {
      double r = std::sqrt(pt.x * pt.x + pt.y * pt.y);
      if (r > minRange && r < maxRange)
        output->points.push_back(pt);
    }
    return output;
  }

  template <typename T>
  static CloudPtr<T>
  filterRange3D(const CloudPtr<T> &input, double minRange, double maxRange) {
    if (input->empty())
      return input;
    auto output = boost::make_shared<Cloud<T>>();
    output->header = input->header;
    for (const auto &pt : input->points) {
      double r = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (r > minRange && r < maxRange)
        output->points.push_back(pt);
    }
    return output;
  }

  template <typename T>
  static CloudPtr<T> filterAngle2D(const CloudPtr<T> &input,
                                   double startDeg,
                                   double endDeg,
                                   bool invert = false) {
    if (input->empty())
      return input;

    auto output = boost::make_shared<Cloud<T>>();
    output->header = input->header;

    const double sRad = normalizeAngle(startDeg) * M_PI / 180.0;
    const double eRad = normalizeAngle(endDeg) * M_PI / 180.0;

    for (const auto &point : input->points) {
      double a = std::atan2(point.y, point.x);
      bool keep = (sRad <= eRad) ? (a >= sRad && a <= eRad) : (a >= sRad || a <= eRad);
      if (invert)
        keep = !keep;
      if (keep)
        output->points.push_back(point);
    }
    return output;
  }

  template <typename T>
  static CloudPtr<T>
  downsampleVoxel(const CloudPtr<T> &input, double dx, double dy, double dz) {
    if (input->empty())
      return input;
    auto output = boost::make_shared<Cloud<T>>();
    pcl::VoxelGrid<T> vg;
    vg.setInputCloud(input);
    vg.setLeafSize(dx, dy, dz);
    vg.filter(*output);
    output->header = input->header;
    return output;
  }

  template <typename T>
  static CloudPtr<T> downsampleVoxel(const CloudPtr<T> &input, double leafSize) {
    return downsampleVoxel<T>(input, leafSize, leafSize, leafSize);
  }

private:
  static double normalizeAngle(double angle) {
    while (angle > 180.0)
      angle -= 360.0;
    while (angle < -180.0)
      angle += 360.0;
    return angle;
  }

  // Prevent instantiation
  PointCloudOps() = delete;
  ~PointCloudOps() = delete;
  PointCloudOps(const PointCloudOps &) = delete;
  PointCloudOps &operator=(const PointCloudOps &) = delete;
};