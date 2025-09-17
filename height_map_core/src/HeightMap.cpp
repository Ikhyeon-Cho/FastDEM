/*
 * HeightMap.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_map_core/data/height_map.h"

namespace height_mapping {

HeightMap::HeightMap() {
  // Add core layers
  add(layers::Height::ELEVATION);
  add(layers::Height::ELEVATION_MIN);
  add(layers::Height::ELEVATION_MAX);
  add(layers::Height::ELEVATION_VARIANCE);
  add(layers::Height::N_MEASUREMENTS, 0.0f);
  setFrameId("map");
  setBasicLayers({layers::Height::ELEVATION,
                  layers::Height::ELEVATION_MIN,
                  layers::Height::ELEVATION_MAX});
}

void HeightMap::addLayer(const std::string &layer, float default_val) {
  if (!exists(layer))
    add(layer, default_val);
}

void HeightMap::removeLayer(const std::string &layer) {
  if (exists(layer))
    erase(layer);
}

void HeightMap::addBasicLayer(const std::string &layer) {
  auto basic_layers = getBasicLayers();
  basic_layers.insert(basic_layers.end(), {layer});
  setBasicLayers(basic_layers);
}

bool HeightMap::hasHeightValues() const {
  const auto &mat = getHeightMatrix();
  auto allNaN = mat.array().unaryExpr([](float elem) { return std::isnan(elem); }).all();
  return !allNaN;
}

std::vector<grid_map::Position3>
HeightMap::getNeighborHeights(const grid_map::Index &index, double radius) const {

  std::vector<grid_map::Position3> neighbors;
  grid_map::Position center_position;
  getPosition(index, center_position);
  grid_map::Position3 position;
  grid_map::CircleIterator iterator(*this, center_position, radius * 1.414);
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    if (getPosition3(layers::Height::ELEVATION, *iterator, position))
      neighbors.push_back(position);
  }
  return neighbors;
}

float HeightMap::getMaxHeight() const {
  return HeightMapMath::getMaxVal(*this, layers::Height::ELEVATION);
}

float HeightMap::getMinHeight() const {
  return HeightMapMath::getMinVal(*this, layers::Height::ELEVATION);
}

float HeightMapMath::getMinVal(const HeightMap &map, const std::string &layer) {
  const auto &data = map[layer];

  auto fillNaNForFindingMinVal =
      data.array().isNaN().select(std::numeric_limits<double>::max(), data);
  return fillNaNForFindingMinVal.minCoeff();
}

float HeightMapMath::getMaxVal(const HeightMap &map, const std::string &layer) {
  const auto &data = map[layer];

  // https://www.geeksforgeeks.org/difference-between-stdnumeric_limitst-min-max-and-lowest-in-cpp/
  auto fillNaNForFindingMaxVal =
      data.array().isNaN().select(std::numeric_limits<double>::lowest(), data);
  return fillNaNForFindingMaxVal.maxCoeff();
}

} // namespace height_mapping
