/*
 * KalmanEstimator.cpp
 *
 *  Created on: Apr 2, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_mapping/algorithms/estimators/KalmanEstimator.h"

namespace height_mapping {

void KalmanEstimator::estimate(HeightMap &map,
                               const pcl::PointCloud<pcl::PointXYZ> &cloud) {

  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId()) {
    std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a "
                 "different frame! \n";
    return;
  }

  auto &heightMatrix = map.getHeightMatrix();
  auto &heightMinMatrix = map.getHeightMinMatrix();
  auto &heightMaxMatrix = map.getHeightMaxMatrix();
  auto &heightVarianceMatrix = map.getHeightVarianceMatrix();
  auto &numMeasuredMatrix = map.getMeasurementCountMatrix();

  map.addLayer(layers::Confidence::CONFIDENCE, 0.0f);
  auto &confidenceMatrix = map[layers::Confidence::CONFIDENCE];

  grid_map::Index measuredIndex;
  grid_map::Position measuredPosition;

  for (const auto &newPoint : cloud) {
    measuredPosition << newPoint.x, newPoint.y;
    if (!map.getIndex(measuredPosition, measuredIndex))
      continue;

    auto &height = heightMatrix(measuredIndex(0), measuredIndex(1));
    auto &variance = heightVarianceMatrix(measuredIndex(0), measuredIndex(1));
    auto &minHeight = heightMinMatrix(measuredIndex(0), measuredIndex(1));
    auto &maxHeight = heightMaxMatrix(measuredIndex(0), measuredIndex(1));
    auto &nPoints = numMeasuredMatrix(measuredIndex(0), measuredIndex(1));
    auto &confidence = confidenceMatrix(measuredIndex(0), measuredIndex(1));

    const Eigen::Vector3f pointVec(newPoint.x, newPoint.y, newPoint.z);
    const float pointVariance = getPointVariance(pointVec);

    if (map.isEmptyAt(measuredIndex)) {
      height = newPoint.z;
      minHeight = newPoint.z;
      maxHeight = newPoint.z;
      variance = pointVariance;
      nPoints = 1;
      confidence = getConfidence(variance);
      continue;
    }

    ++nPoints;
    kalmanUpdate(height, variance, newPoint.z, pointVariance);
    minHeight = std::min(minHeight, newPoint.z);
    maxHeight = std::max(maxHeight, newPoint.z);
    confidence = getConfidence(variance);
  }
}

void KalmanEstimator::estimate(HeightMap &map,
                               const pcl::PointCloud<pcl::PointXYZI> &cloud) {

  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId()) {
    std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a "
                 "different frame! \n";
    return;
  }

  auto &heightMatrix = map.getHeightMatrix();
  auto &heightVarianceMatrix = map.getHeightVarianceMatrix();
  auto &heightMinMatrix = map.getHeightMinMatrix();
  auto &heightMaxMatrix = map.getHeightMaxMatrix();

  map.addLayer(layers::Confidence::CONFIDENCE, 0.0f);
  map.addLayer(layers::Sensor::Lidar::INTENSITY);
  auto &confidenceMatrix = map[layers::Confidence::CONFIDENCE];
  auto &intensityMatrix = map[layers::Sensor::Lidar::INTENSITY];

  grid_map::Index measuredIndex;
  grid_map::Position measuredPosition;

  for (const auto &newPoint : cloud) {
    measuredPosition << newPoint.x, newPoint.y;
    if (!map.getIndex(measuredPosition, measuredIndex))
      continue;

    auto &height = heightMatrix(measuredIndex(0), measuredIndex(1));
    auto &variance = heightVarianceMatrix(measuredIndex(0), measuredIndex(1));
    auto &minHeight = heightMinMatrix(measuredIndex(0), measuredIndex(1));
    auto &maxHeight = heightMaxMatrix(measuredIndex(0), measuredIndex(1));
    auto &confidence = confidenceMatrix(measuredIndex(0), measuredIndex(1));
    auto &intensity = intensityMatrix(measuredIndex(0), measuredIndex(1));

    const Eigen::Vector3f pointVec(newPoint.x, newPoint.y, newPoint.z);
    const float pointVariance = getPointVariance(pointVec);

    if (map.isEmptyAt(measuredIndex)) {
      height = newPoint.z;
      variance = pointVariance;
      minHeight = maxHeight = newPoint.z;
      intensity = newPoint.intensity;
      confidence = getConfidence(variance);
      continue;
    }

    kalmanUpdate(height, variance, newPoint.z, pointVariance);
    kalmanUpdate(intensity, variance, newPoint.intensity, pointVariance);
    minHeight = std::min(minHeight, newPoint.z);
    maxHeight = std::max(maxHeight, newPoint.z);
    confidence = getConfidence(variance);
  }
}

void KalmanEstimator::estimate(HeightMap &map,
                               const pcl::PointCloud<pcl::PointXYZRGB> &cloud) {

  if (hasEmptyCloud(cloud))
    return;

  if (cloud.header.frame_id != map.getFrameId()) {
    std::cout << "[HeightEstimator]: Frame ID mismatch - pointcloud is in a "
                 "different frame! \n";
    return;
  }

  auto &heightMatrix = map.getHeightMatrix();
  auto &heightVarianceMatrix = map.getHeightVarianceMatrix();
  auto &heightMinMatrix = map.getHeightMinMatrix();
  auto &heightMaxMatrix = map.getHeightMaxMatrix();

  map.addLayer(layers::Confidence::CONFIDENCE, 0.0f);
  map.addLayer(layers::Sensor::RGBD::R);
  map.addLayer(layers::Sensor::RGBD::G);
  map.addLayer(layers::Sensor::RGBD::B);
  map.addLayer(layers::Sensor::RGBD::COLOR);
  auto &confidenceMatrix = map[layers::Confidence::CONFIDENCE];
  auto &redMatrix = map[layers::Sensor::RGBD::R];
  auto &greenMatrix = map[layers::Sensor::RGBD::G];
  auto &blueMatrix = map[layers::Sensor::RGBD::B];
  auto &colorMatrix = map[layers::Sensor::RGBD::COLOR];

  grid_map::Index measuredIndex;
  grid_map::Position measuredPosition;

  for (const auto &newPoint : cloud) {
    measuredPosition << newPoint.x, newPoint.y;
    if (!map.getIndex(measuredPosition, measuredIndex))
      continue;

    auto &height = heightMatrix(measuredIndex(0), measuredIndex(1));
    auto &variance = heightVarianceMatrix(measuredIndex(0), measuredIndex(1));
    auto &minHeight = heightMinMatrix(measuredIndex(0), measuredIndex(1));
    auto &maxHeight = heightMaxMatrix(measuredIndex(0), measuredIndex(1));
    auto &confidence = confidenceMatrix(measuredIndex(0), measuredIndex(1));
    auto &red = redMatrix(measuredIndex(0), measuredIndex(1));
    auto &green = greenMatrix(measuredIndex(0), measuredIndex(1));
    auto &blue = blueMatrix(measuredIndex(0), measuredIndex(1));
    auto &color = colorMatrix(measuredIndex(0), measuredIndex(1));

    const Eigen::Vector3f pointVec(newPoint.x, newPoint.y, newPoint.z);
    const float pointVariance = getPointVariance(pointVec);

    if (map.isEmptyAt(measuredIndex)) {
      height = newPoint.z;
      variance = pointVariance;
      minHeight = maxHeight = newPoint.z;
      red = newPoint.r;
      green = newPoint.g;
      blue = newPoint.b;
      grid_map::colorVectorToValue(newPoint.getRGBVector3i(), color);
      confidence = getConfidence(variance);
      continue;
    }

    kalmanUpdate(height, variance, newPoint.z, pointVariance);
    kalmanUpdate(red, variance, newPoint.r, pointVariance);
    kalmanUpdate(green, variance, newPoint.g, pointVariance);
    kalmanUpdate(blue, variance, newPoint.b, pointVariance);
    grid_map::colorVectorToValue(Eigen::Vector3i(red, green, blue), color);
    minHeight = std::min(minHeight, newPoint.z);
    maxHeight = std::max(maxHeight, newPoint.z);
    confidence = getConfidence(variance);
  }
}

} // namespace height_mapping