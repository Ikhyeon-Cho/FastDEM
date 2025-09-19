/*
 * height_map.cpp
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "height_map_core/height_map.h"
#include <cmath>

namespace height_map {

HeightMap::HeightMap() : map_(std::make_unique<grid_map::GridMap>()) {
  // Initialize with core layers
  map_->add(Layers::ELEVATION);
  map_->add(Layers::VARIANCE, 1000.0f); // High initial variance
  map_->setBasicLayers({Layers::ELEVATION});
}

void HeightMap::initialize(float width, float height, float resolution) {
  map_->setGeometry(grid_map::Length(width, height), resolution);
  map_->clearAll();

  // Initialize variance to high value
  map_->get(Layers::VARIANCE).setConstant(1000.0f);
}

void HeightMap::setFrameId(const std::string &frame_id) {
  map_->setFrameId(frame_id);
}

std::string HeightMap::getFrameId() const { return map_->getFrameId(); }

bool HeightMap::isInitialized() const {
  // Map is initialized if it has non-zero size
  const auto &size = map_->getSize();
  return size(0) > 0 && size(1) > 0;
}

void HeightMap::getSize(float &width, float &height) const {
  const auto &length = map_->getLength();
  width = length.x();
  height = length.y();
}

bool HeightMap::isEmpty() const {
  // Map is empty if all elevation values are NaN (uninitialized)
  const auto &elevation = map_->get(Layers::ELEVATION);
  return elevation.array().isNaN().all();
}

void HeightMap::clear() {
  map_->clearAll();
  map_->get(Layers::VARIANCE).setConstant(1000.0f);
}

// Map origin management
void HeightMap::move(const grid_map::Position &position) {
  map_->move(position);
}

void HeightMap::setPosition(const grid_map::Position &position) {
  map_->setPosition(position);
}

grid_map::Position HeightMap::getPosition() const {
  return map_->getPosition();
}

grid_map::Length HeightMap::getLength() const { return map_->getLength(); }

bool HeightMap::setHeight(float x, float y, float height) {
  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return false;

  map_->atPosition(Layers::ELEVATION, position) = height;
  return true;
}

bool HeightMap::setHeight(float x, float y, float height, float variance) {
  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return false;

  map_->atPosition(Layers::ELEVATION, position) = height;
  map_->atPosition(Layers::VARIANCE, position) = variance;
  return true;
}

bool HeightMap::getHeight(float x, float y, float &height) const {
  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return false;

  height = map_->atPosition(Layers::ELEVATION, position);
  return std::isfinite(height);
}

bool HeightMap::setVariance(float x, float y, float variance) {
  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return false;

  if (!map_->exists(Layers::VARIANCE)) {
    map_->add(Layers::VARIANCE, 1000.0f);
  }

  map_->atPosition(Layers::VARIANCE, position) = variance;
  return true;
}

bool HeightMap::getVariance(float x, float y, float &variance) const {
  if (!map_->exists(Layers::VARIANCE))
    return false;

  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return false;

  variance = map_->atPosition(Layers::VARIANCE, position);
  return std::isfinite(variance);
}

bool HeightMap::updateMinMax(float x, float y, float height) {
  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return false;

  // Add layers if they don't exist
  if (!map_->exists(Layers::MIN_HEIGHT)) {
    map_->add(Layers::MIN_HEIGHT, NAN);
  }
  if (!map_->exists(Layers::MAX_HEIGHT)) {
    map_->add(Layers::MAX_HEIGHT, NAN);
  }

  float &min_val = map_->atPosition(Layers::MIN_HEIGHT, position);
  float &max_val = map_->atPosition(Layers::MAX_HEIGHT, position);

  if (!std::isfinite(min_val) || height < min_val) {
    min_val = height;
  }
  if (!std::isfinite(max_val) || height > max_val) {
    max_val = height;
  }

  return true;
}

bool HeightMap::getMinHeight(float x, float y, float &min_height) const {
  if (!map_->exists(Layers::MIN_HEIGHT))
    return false;

  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return false;

  min_height = map_->atPosition(Layers::MIN_HEIGHT, position);
  return std::isfinite(min_height);
}

bool HeightMap::getMaxHeight(float x, float y, float &max_height) const {
  if (!map_->exists(Layers::MAX_HEIGHT))
    return false;

  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return false;

  max_height = map_->atPosition(Layers::MAX_HEIGHT, position);
  return std::isfinite(max_height);
}

bool HeightMap::incrementMeasurementCount(float x, float y) {
  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return false;

  if (!map_->exists(Layers::MEASUREMENT_COUNT)) {
    map_->add(Layers::MEASUREMENT_COUNT, 0.0f);
  }

  map_->atPosition(Layers::MEASUREMENT_COUNT, position) += 1.0f;
  return true;
}

bool HeightMap::getMeasurementCount(float x, float y, int &count) const {
  if (!map_->exists(Layers::MEASUREMENT_COUNT)) {
    count = 0;
    return false;
  }

  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return false;

  float value = map_->atPosition(Layers::MEASUREMENT_COUNT, position);
  count = static_cast<int>(value);
  return true;
}

bool HeightMap::setTraversability(float x, float y, float value) {
  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return false;

  if (!map_->exists(Layers::TRAVERSABILITY)) {
    map_->add(Layers::TRAVERSABILITY, 0.0f);
  }

  map_->atPosition(Layers::TRAVERSABILITY, position) = value;
  return true;
}

bool HeightMap::getTraversability(float x, float y, float &value) const {
  if (!map_->exists(Layers::TRAVERSABILITY))
    return false;

  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return false;

  value = map_->atPosition(Layers::TRAVERSABILITY, position);
  return std::isfinite(value);
}

bool HeightMap::isTraversable(float x, float y, float threshold) const {
  float value;
  if (!getTraversability(x, y, value))
    return false;
  return value > threshold;
}

float HeightMap::getInterpolatedHeight(float x, float y) const {
  grid_map::Position position(x, y);
  if (!map_->isInside(position))
    return NAN;

  return map_->atPosition(Layers::ELEVATION, position,
                          grid_map::InterpolationMethods::INTER_LINEAR);
}

bool HeightMap::computeNormalVector(float x, float y,
                                    Eigen::Vector3f &normal) const {
  grid_map::Position position(x, y);
  grid_map::Index index;

  if (!map_->getIndex(position, index))
    return false;

  // Get neighboring heights for gradient computation
  const auto &elevation = map_->get(Layers::ELEVATION);
  const float res = map_->getResolution();

  // Check bounds
  if (index(0) <= 0 || index(0) >= elevation.rows() - 1 || index(1) <= 0 ||
      index(1) >= elevation.cols() - 1) {
    return false;
  }

  // Compute gradients using central differences
  float dz_dx =
      (elevation(index(0) + 1, index(1)) - elevation(index(0) - 1, index(1))) /
      (2.0f * res);
  float dz_dy =
      (elevation(index(0), index(1) + 1) - elevation(index(0), index(1) - 1)) /
      (2.0f * res);

  // Normal vector: (-dz/dx, -dz/dy, 1) normalized
  normal = Eigen::Vector3f(-dz_dx, -dz_dy, 1.0f);
  normal.normalize();

  return true;
}

float HeightMap::computeSlope(float x, float y) const {
  Eigen::Vector3f normal;
  if (!computeNormalVector(x, y, normal))
    return NAN;

  // Slope is the angle between normal and vertical
  // cos(slope) = normal.z (since normal is normalized)
  return std::acos(normal.z());
}

float HeightMap::computeRoughness(float x, float y, float radius) const {
  grid_map::Position center(x, y);

  // Create circular iterator around the point
  grid_map::CircleIterator iterator(*map_, center, radius);

  std::vector<float> heights;
  heights.reserve(50); // Approximate

  for (; !iterator.isPastEnd(); ++iterator) {
    const float &h = map_->at(Layers::ELEVATION, *iterator);
    if (std::isfinite(h)) {
      heights.push_back(h);
    }
  }

  if (heights.size() < 3)
    return NAN;

  // Compute standard deviation as roughness measure
  float mean = 0.0f;
  for (float h : heights)
    mean += h;
  mean /= heights.size();

  float variance = 0.0f;
  for (float h : heights) {
    float diff = h - mean;
    variance += diff * diff;
  }
  variance /= heights.size();

  return std::sqrt(variance);
}

bool HeightMap::hasLayer(const std::string &layer) const {
  return map_->exists(layer);
}

void HeightMap::addLayer(const std::string &layer, float default_value) {
  if (!map_->exists(layer)) {
    map_->add(layer, default_value);
  }
}

void HeightMap::removeLayer(const std::string &layer) {
  if (map_->exists(layer)) {
    map_->erase(layer);
  }
}

std::vector<std::string> HeightMap::getLayers() const {
  return map_->getLayers();
}

Eigen::MatrixXf &HeightMap::getElevationMatrix() {
  return map_->get(Layers::ELEVATION);
}

const Eigen::MatrixXf &HeightMap::getElevationMatrix() const {
  return map_->get(Layers::ELEVATION);
}

Eigen::MatrixXf &HeightMap::getVarianceMatrix() {
  if (!map_->exists(Layers::VARIANCE)) {
    map_->add(Layers::VARIANCE, 1000.0f);
  }
  return map_->get(Layers::VARIANCE);
}

const Eigen::MatrixXf &HeightMap::getVarianceMatrix() const {
  return map_->get(Layers::VARIANCE);
}

bool HeightMap::getIndex(float x, float y, grid_map::Index &index) const {
  return map_->getIndex(grid_map::Position(x, y), index);
}

bool HeightMap::getPosition(const grid_map::Index &index, float &x,
                            float &y) const {
  grid_map::Position position;
  if (!map_->getPosition(index, position))
    return false;
  x = position.x();
  y = position.y();
  return true;
}

grid_map::GridMapIterator HeightMap::begin() const {
  return grid_map::GridMapIterator(*map_);
}

grid_map::GridMapIterator HeightMap::end() const {
  // Return a past-the-end iterator
  grid_map::GridMapIterator iterator(*map_);
  // Move iterator to the end
  while (!iterator.isPastEnd()) {
    ++iterator;
  }
  return iterator;
}

HeightMap HeightMap::getSubmap(float center_x, float center_y, float width,
                               float height) const {
  HeightMap submap;

  // Extract submap using grid_map functionality
  grid_map::Position center(center_x, center_y);
  grid_map::Length length(width, height);
  bool success;

  *submap.map_ = map_->getSubmap(center, length, success);

  return submap;
}

HeightMap::Statistics HeightMap::computeStatistics() const {
  Statistics stats;
  stats.min_height = std::numeric_limits<float>::max();
  stats.max_height = std::numeric_limits<float>::lowest();
  stats.mean_height = 0.0f;
  stats.std_dev = 0.0f;
  stats.valid_cells = 0;
  stats.total_cells = map_->getSize().prod();

  const auto &elevation = map_->get(Layers::ELEVATION);

  // First pass: compute min, max, mean
  float sum = 0.0f;
  for (int i = 0; i < elevation.size(); ++i) {
    const float &h = elevation.data()[i];
    if (std::isfinite(h)) {
      stats.min_height = std::min(stats.min_height, h);
      stats.max_height = std::max(stats.max_height, h);
      sum += h;
      stats.valid_cells++;
    }
  }

  if (stats.valid_cells > 0) {
    stats.mean_height = sum / stats.valid_cells;

    // Second pass: compute standard deviation
    float sum_sq_diff = 0.0f;
    for (int i = 0; i < elevation.size(); ++i) {
      const float &h = elevation.data()[i];
      if (std::isfinite(h)) {
        float diff = h - stats.mean_height;
        sum_sq_diff += diff * diff;
      }
    }
    stats.std_dev = std::sqrt(sum_sq_diff / stats.valid_cells);
  }

  return stats;
}

bool HeightMap::getIndexSafe(float x, float y, grid_map::Index &index) const {
  return map_->getIndex(grid_map::Position(x, y), index);
}

// Additional matrix access methods
Eigen::MatrixXf &HeightMap::getHeightMinMatrix() {
  if (!map_->exists(Layers::MIN_HEIGHT)) {
    map_->add(Layers::MIN_HEIGHT, std::numeric_limits<float>::max());
  }
  return map_->get(Layers::MIN_HEIGHT);
}

const Eigen::MatrixXf &HeightMap::getHeightMinMatrix() const {
  return map_->get(Layers::MIN_HEIGHT);
}

Eigen::MatrixXf &HeightMap::getHeightMaxMatrix() {
  if (!map_->exists(Layers::MAX_HEIGHT)) {
    map_->add(Layers::MAX_HEIGHT, std::numeric_limits<float>::lowest());
  }
  return map_->get(Layers::MAX_HEIGHT);
}

const Eigen::MatrixXf &HeightMap::getHeightMaxMatrix() const {
  return map_->get(Layers::MAX_HEIGHT);
}

Eigen::MatrixXf &HeightMap::getMeasurementCountMatrix() {
  if (!map_->exists(Layers::MEASUREMENT_COUNT)) {
    map_->add(Layers::MEASUREMENT_COUNT, 0.0f);
  }
  return map_->get(Layers::MEASUREMENT_COUNT);
}

const Eigen::MatrixXf &HeightMap::getMeasurementCountMatrix() const {
  return map_->get(Layers::MEASUREMENT_COUNT);
}

bool HeightMap::isEmptyAt(const grid_map::Index &index) const {
  return !map_->isValid(index) ||
         std::isnan(map_->at(Layers::ELEVATION, index));
}

Eigen::MatrixXf &HeightMap::operator[](const std::string &layer) {
  if (!map_->exists(layer)) {
    map_->add(layer);
  }
  return map_->get(layer);
}

const Eigen::MatrixXf &HeightMap::operator[](const std::string &layer) const {
  return map_->get(layer);
}

void HeightMap::clear(const std::string &layer) {
  if (map_->exists(layer)) {
    map_->get(layer).setConstant(NAN);
  }
}

} // namespace height_map