/*
 * HeightMap.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_H
#define HEIGHT_MAP_H

#include <string>
#include <grid_map_core/grid_map_core.hpp>
#include "height_map_core/data/layer_definitions.h"

namespace height_mapping {

class HeightMap : public grid_map::GridMap {
public:
  HeightMap();

  // Layer management
  void addLayer(const std::string &layer, float default_value = NAN);
  void addBasicLayer(const std::string &layer);
  void removeLayer(const std::string &layer);
  bool hasLayer(const std::string &layer) const { return exists(layer); }

  // Core layer accessors
  Matrix &getHeightMatrix() { return get(layers::Height::ELEVATION); }
  Matrix &getHeightMinMatrix() { return get(layers::Height::ELEVATION_MIN); }
  Matrix &getHeightMaxMatrix() { return get(layers::Height::ELEVATION_MAX); }
  Matrix &getHeightVarianceMatrix() { return get(layers::Height::ELEVATION_VARIANCE); }
  Matrix &getMeasurementCountMatrix() { return get(layers::Height::N_MEASUREMENTS); }

  // Core layer accessors: const versions
  const Matrix &getHeightMatrix() const { return get(layers::Height::ELEVATION); }
  const Matrix &getHeightMinMatrix() const { return get(layers::Height::ELEVATION_MIN); }
  const Matrix &getHeightMaxMatrix() const { return get(layers::Height::ELEVATION_MAX); }
  const Matrix &getHeightVarianceMatrix() const {
    return get(layers::Height::ELEVATION_VARIANCE);
  }
  const Matrix &getMeasurementCountMatrix() const {
    return get(layers::Height::N_MEASUREMENTS);
  }

  // Cell validity checks
  bool isEmptyAt(const grid_map::Index &index) const { return !isValid(index); }
  bool isEmptyAt(const std::string &layer, const grid_map::Index &index) const {
    return !exists(layer) || !std::isfinite(at(layer, index));
  }

  // Height statistics
  float getMinHeight() const;
  float getMaxHeight() const;
  bool hasHeightValues() const;

  // Basic spatial functions
  std::vector<grid_map::Position3> getNeighborHeights(const grid_map::Index &index,
                                                      double radius) const;

  bool is_initialized_{false};
};

class HeightMapMath {
public:
  static float getMinVal(const HeightMap &map, const std::string &layer);

  static float getMaxVal(const HeightMap &map, const std::string &layer);
};

} // namespace height_mapping

#endif // HEIGHT_MAP_H