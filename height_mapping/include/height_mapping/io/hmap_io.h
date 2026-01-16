/*
 * hmap_io.h
 *
 * Binary .hmap format for lossless HeightMap serialization.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_IO_HMAP_IO_H
#define HEIGHT_MAPPING_IO_HMAP_IO_H

#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include "height_mapping/core/map.h"

namespace height_mapping {
namespace io {

/**
 * @brief Binary file header for .hmap format
 *
 * Stores complete geometry information including circular buffer state
 * for lossless save/load of HeightMap data.
 */
struct HMapHeader {
  char magic[4] = {'H', 'M', 'A', 'P'};
  uint32_t version = 1;

  // Geometry
  float resolution;
  uint32_t cols, rows;
  float pos_x, pos_y;
  uint32_t start_idx_x, start_idx_y;  // Circular buffer state

  // Metadata
  uint32_t frame_id_len;
  uint32_t num_layers;
};

/**
 * @brief Save HeightMap to binary .hmap file
 *
 * Binary format preserves:
 * - Complete geometry (resolution, size, position)
 * - Circular buffer start index (for robot-centric maps)
 * - Frame ID
 * - All layer data (lossless float storage)
 *
 * @param filename Output file path (.hmap extension recommended)
 * @param map HeightMap to save
 * @return true if successful
 */
bool saveHMap(const std::string& filename, const HeightMap& map);

/**
 * @brief Load HeightMap from binary .hmap file
 *
 * @param filename Input file path
 * @param map [out] HeightMap to load into (will be cleared and resized)
 * @return true if successful
 */
bool loadHMap(const std::string& filename, HeightMap& map);

// =============================================================================
// Inline Implementation
// =============================================================================

inline bool saveHMap(const std::string& filename, const HeightMap& map) {
  std::ofstream fs(filename, std::ios::binary);
  if (!fs.is_open()) {
    std::cerr << "[hmap_io] Error: Cannot create " << filename << std::endl;
    return false;
  }

  // Prepare header
  HMapHeader header;
  header.resolution = static_cast<float>(map.getResolution());
  header.cols = static_cast<uint32_t>(map.getSize()(1));
  header.rows = static_cast<uint32_t>(map.getSize()(0));
  header.pos_x = static_cast<float>(map.getPosition().x());
  header.pos_y = static_cast<float>(map.getPosition().y());

  auto start_idx = map.getStartIndex();
  header.start_idx_x = static_cast<uint32_t>(start_idx(0));
  header.start_idx_y = static_cast<uint32_t>(start_idx(1));

  const std::string& frame_id = map.getFrameId();
  header.frame_id_len = static_cast<uint32_t>(frame_id.size());

  const auto& layers = map.getLayers();
  header.num_layers = static_cast<uint32_t>(layers.size());

  // Write header
  fs.write(reinterpret_cast<const char*>(&header), sizeof(HMapHeader));

  // Write frame_id
  fs.write(frame_id.c_str(), frame_id.size());

  // Write each layer
  for (const auto& layer_name : layers) {
    // Write layer name (length + string)
    uint32_t name_len = static_cast<uint32_t>(layer_name.size());
    fs.write(reinterpret_cast<const char*>(&name_len), sizeof(uint32_t));
    fs.write(layer_name.c_str(), layer_name.size());

    // Write layer data (row-major float array)
    const auto& matrix = map.get(layer_name);
    fs.write(reinterpret_cast<const char*>(matrix.data()),
             matrix.size() * sizeof(float));
  }

  return true;
}

inline bool loadHMap(const std::string& filename, HeightMap& map) {
  std::ifstream fs(filename, std::ios::binary);
  if (!fs.is_open()) {
    std::cerr << "[hmap_io] Error: Cannot open " << filename << std::endl;
    return false;
  }

  // Read header
  HMapHeader header;
  fs.read(reinterpret_cast<char*>(&header), sizeof(HMapHeader));

  // Validate magic
  if (std::strncmp(header.magic, "HMAP", 4) != 0) {
    std::cerr << "[hmap_io] Error: Invalid file format (bad magic)"
              << std::endl;
    return false;
  }

  if (header.version != 1) {
    std::cerr << "[hmap_io] Error: Unsupported version " << header.version
              << std::endl;
    return false;
  }

  // Read frame_id
  std::string frame_id(header.frame_id_len, '\0');
  fs.read(&frame_id[0], header.frame_id_len);

  // Initialize map geometry
  // Note: We need to set geometry first, then set start index
  float length_x = header.resolution * header.cols;
  float length_y = header.resolution * header.rows;

  map.setFrameId(frame_id);
  map.setGeometry(length_x, length_y, header.resolution);
  map.setPosition(grid_map::Position(header.pos_x, header.pos_y));
  map.setStartIndex(grid_map::Index(header.start_idx_x, header.start_idx_y));

  // Read layers
  for (uint32_t i = 0; i < header.num_layers; ++i) {
    // Read layer name
    uint32_t name_len;
    fs.read(reinterpret_cast<char*>(&name_len), sizeof(uint32_t));
    std::string layer_name(name_len, '\0');
    fs.read(&layer_name[0], name_len);

    // Add layer if it doesn't exist
    if (!map.exists(layer_name)) {
      map.add(layer_name);
    }

    // Read layer data
    auto& matrix = map.get(layer_name);
    fs.read(reinterpret_cast<char*>(matrix.data()),
            matrix.size() * sizeof(float));
  }

  return true;
}

}  // namespace io
}  // namespace height_mapping

#endif  // HEIGHT_MAPPING_IO_HMAP_IO_H
