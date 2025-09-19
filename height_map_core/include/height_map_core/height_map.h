/*
 * height_map.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAP_CORE_DATA_HEIGHT_MAP_H
#define HEIGHT_MAP_CORE_DATA_HEIGHT_MAP_H

#include <grid_map_core/grid_map_core.hpp>
#include <memory>
#include <shared_mutex>
#include <string>

namespace height_map {

// Height map wrapper around grid_map for domain-specific operations
class HeightMap {
private:
  std::unique_ptr<grid_map::GridMap> map_;

  // Core layer names
  struct Layers {
    static constexpr const char *ELEVATION = "elevation";
    static constexpr const char *VARIANCE = "variance";
    static constexpr const char *MIN_HEIGHT = "min_height";
    static constexpr const char *MAX_HEIGHT = "max_height";
    static constexpr const char *MEASUREMENT_COUNT = "measurement_count";
    static constexpr const char *TRAVERSABILITY = "traversability";
  };

public:
  HeightMap();
  ~HeightMap() = default;

  // Move semantics
  HeightMap(HeightMap &&) = default;
  HeightMap &operator=(HeightMap &&) = default;

  // Delete copy (expensive operation)
  HeightMap(const HeightMap &) = delete;
  HeightMap &operator=(const HeightMap &) = delete;

  // Initialize map
  void initialize(float width, float height, float resolution);
  void setFrameId(const std::string &frame_id);
  std::string getFrameId() const;
  bool isInitialized() const;

  // Map origin management
  void move(const grid_map::Position &position);
  void setPosition(const grid_map::Position &position);
  grid_map::Position getPosition() const;
  grid_map::Length getLength() const;

  // Map properties
  float getResolution() const;
  void getSize(float &width, float &height) const;
  bool isInBounds(float x, float y) const;
  bool isEmpty() const;
  void clear();

  // Height operations
  bool setHeight(float x, float y, float height);
  bool setHeight(float x, float y, float height, float variance);
  bool getHeight(float x, float y, float &height) const;
  float getHeightSafe(float x, float y) const; // Returns NaN if out of bounds

  // Variance operations
  bool setVariance(float x, float y, float variance);
  bool getVariance(float x, float y, float &variance) const;

  // Min/Max tracking
  bool updateMinMax(float x, float y, float height);
  bool getMinHeight(float x, float y, float &min_height) const;
  bool getMaxHeight(float x, float y, float &max_height) const;

  // Measurement count tracking
  bool incrementMeasurementCount(float x, float y);
  bool getMeasurementCount(float x, float y, int &count) const;

  // Traversability analysis
  bool setTraversability(float x, float y, float value);
  bool getTraversability(float x, float y, float &value) const;
  bool isTraversable(float x, float y, float threshold = 0.5) const;

  // Advanced operations
  float getInterpolatedHeight(float x, float y) const;
  bool computeNormalVector(float x, float y, Eigen::Vector3f &normal) const;
  float computeSlope(float x, float y) const;
  float computeRoughness(float x, float y, float radius = 0.1) const;

  // Layer management
  bool hasLayer(const std::string &layer) const;
  void addLayer(const std::string &layer, float default_value = NAN);
  void removeLayer(const std::string &layer);
  std::vector<std::string> getLayers() const;

  // Direct matrix access (for algorithms that need it)
  Eigen::MatrixXf &getElevationMatrix();
  const Eigen::MatrixXf &getElevationMatrix() const;
  Eigen::MatrixXf &getVarianceMatrix();
  const Eigen::MatrixXf &getVarianceMatrix() const;

  // Compatibility methods for height_mapping_core
  Eigen::MatrixXf &getHeightMatrix() { return getElevationMatrix(); }
  const Eigen::MatrixXf &getHeightMatrix() const {
    return getElevationMatrix();
  }
  Eigen::MatrixXf &getHeightVarianceMatrix() { return getVarianceMatrix(); }
  const Eigen::MatrixXf &getHeightVarianceMatrix() const {
    return getVarianceMatrix();
  }

  // Matrix access for additional layers
  Eigen::MatrixXf &getHeightMinMatrix();
  const Eigen::MatrixXf &getHeightMinMatrix() const;
  Eigen::MatrixXf &getHeightMaxMatrix();
  const Eigen::MatrixXf &getHeightMaxMatrix() const;
  Eigen::MatrixXf &getMeasurementCountMatrix();
  const Eigen::MatrixXf &getMeasurementCountMatrix() const;

  // Check if cell is empty
  bool isEmptyAt(const grid_map::Index &index) const;

  // Layer access operators
  Eigen::MatrixXf &operator[](const std::string &layer);
  const Eigen::MatrixXf &operator[](const std::string &layer) const;

  // Clear layer
  void clear(const std::string &layer);

  // Grid index operations
  bool getIndex(float x, float y, grid_map::Index &index) const;
  bool getPosition(const grid_map::Index &index, float &x, float &y) const;

  // Batch processing support - get indices for multiple points
  std::vector<grid_map::Index>
  getIndices(const std::vector<std::pair<float, float>> &positions) const;

  // Iterators (leverage grid_map's powerful iterators)
  grid_map::GridMapIterator begin() const;
  grid_map::GridMapIterator end() const;

  // Submap operations
  HeightMap getSubmap(float center_x, float center_y, float width,
                      float height) const;

  // Statistics
  struct Statistics {
    float min_height;
    float max_height;
    float mean_height;
    float std_dev;
    size_t valid_cells;
    size_t total_cells;
  };
  Statistics computeStatistics() const;

  // Access underlying grid_map (for ROS conversion)
  const grid_map::GridMap &getGridMap() const { return *map_; }
  grid_map::GridMap &getGridMap() { return *map_; }

private:
  // Helper to check if position is valid and get index
  bool getIndexSafe(float x, float y, grid_map::Index &index) const;
};

// Inline implementations for performance-critical functions
inline bool HeightMap::isInBounds(float x, float y) const {
  return map_->isInside(grid_map::Position(x, y));
}

inline float HeightMap::getHeightSafe(float x, float y) const {
  float height;
  return getHeight(x, y, height) ? height : NAN;
}

inline float HeightMap::getResolution() const { return map_->getResolution(); }

} // namespace height_map

#endif // HEIGHT_MAP_CORE_DATA_HEIGHT_MAP_H