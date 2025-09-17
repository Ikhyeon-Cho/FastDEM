/*
 * HeightMapRaycaster.h
 *
 *  Created on: Nov 28, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include "height_map_core/HeightMap.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace height_mapping {

class HeightMapRaycaster {
public:
  HeightMapRaycaster() = default;

  template <typename PointT>
  void correctHeight(HeightMap &map,
                     const pcl::PointCloud<PointT> &cloud,
                     const Eigen::Vector3f &sensorOrigin) {

    auto &heightMatrix = map.getHeightMatrix();
    auto &maxHeightMatrix = map.getHeightMaxMatrix();
    auto &varianceMatrix = map.getHeightVarianceMatrix();
    auto &numMeasuredMatrix = map.getMeasurementCountMatrix();

    map.addLayer(layers::Scan::RAY_CASTING);
    map.clear(layers::Scan::RAY_CASTING);
    auto &raycastingMatrix = map.get(layers::Scan::RAY_CASTING);

    map.addLayer(layers::Scan::SCAN_HEIGHT);
    map.clear(layers::Scan::SCAN_HEIGHT);
    auto &scanHeightMatrix = map.get(layers::Scan::SCAN_HEIGHT);

    const float sensorHeight = sensorOrigin.z();

    grid_map::Index measuredIndex;
    grid_map::Position measuredPosition;

    // Record current scan heights to the map
    for (const auto &point : cloud.points) {
      measuredPosition << point.x, point.y;
      if (!map.getIndex(measuredPosition, measuredIndex))
        continue;

      scanHeightMatrix(measuredIndex(0), measuredIndex(1)) = point.z;
    }

    // Raycasting loop
    for (const auto &point : cloud.points) {
      // Create ray from two points: sensor to measured point
      Eigen::Vector3f rayDir(point.x - sensorOrigin.x(),
                             point.y - sensorOrigin.y(),
                             point.z - sensorOrigin.z());
      float rayLength = rayDir.norm();
      rayDir.normalize();

      // Visibility check through ray
      float samplingStep = map.getResolution();
      for (float t = 0; t < rayLength - samplingStep; t += samplingStep) {

        // Get ray point: starting from sensor
        Eigen::Vector3f pointOnRay = sensorOrigin + rayDir * t;

        // Get ray point index
        grid_map::Position checkPosition(pointOnRay.x(), pointOnRay.y());
        grid_map::Index checkIndex;
        // Skip if the ray point is out of the map
        if (!map.getIndex(checkPosition, checkIndex))
          continue;

        // Do not erase the static obstacles
        auto &scanHeight = scanHeightMatrix(checkIndex(0), checkIndex(1));
        if (std::isfinite(scanHeight) && scanHeight > pointOnRay.z() + 0.1)
          break;

        // Get map height and variance at the ray point
        auto &mapHeight = heightMatrix(checkIndex(0), checkIndex(1));
        auto &mapMaxHeight = maxHeightMatrix(checkIndex(0), checkIndex(1));
        auto &mapHeightVariance = varianceMatrix(checkIndex(0), checkIndex(1));
        auto &rayHeight = raycastingMatrix(checkIndex(0), checkIndex(1));
        auto &nPoints = numMeasuredMatrix(checkIndex(0), checkIndex(1));

        // For visualization of traced ray
        if (!std::isfinite(rayHeight))
          rayHeight = pointOnRay.z();
        else if (rayHeight > pointOnRay.z())
          rayHeight = pointOnRay.z();

        // Update height if current height is higher than the ray point
        if (mapHeight > pointOnRay.z() + correctionThreshold_) {
          mapHeightVariance += (mapHeight - pointOnRay.z()); // Increase variance
          nPoints = 1;                                       // Reset nPoints
          mapHeight = pointOnRay.z() + correctionThreshold_; // Height correction
          // mapMaxHeight =
          //     pointOnRay.z() + correctionThreshold_; // Update max height
        }
      }
    }
  }

private:
  float correctionThreshold_{0.02f};
};

} // namespace height_mapping
