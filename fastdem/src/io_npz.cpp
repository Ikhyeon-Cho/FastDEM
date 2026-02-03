// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024 Ikhyeon Cho <tre0430@korea.ac.kr>

/*
 * npz_io.cpp
 *
 *  Created on: Feb 2025
 *      Author: Ikhyeon Cho
 *   Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "fastdem/io/npz.hpp"

#include <spdlog/spdlog.h>

#include <array>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace fastdem {
namespace io {

namespace detail {

// ─── CRC32 ──────────────────────────────────────────────────────────────────

constexpr std::array<uint32_t, 256> buildCrc32Table() {
  std::array<uint32_t, 256> table{};
  for (uint32_t i = 0; i < 256; ++i) {
    uint32_t c = i;
    for (int j = 0; j < 8; ++j) {
      c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
    table[i] = c;
  }
  return table;
}

static constexpr auto crc32Table = buildCrc32Table();

uint32_t crc32(const void* data, size_t len) {
  const auto* buf = static_cast<const uint8_t*>(data);
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc = crc32Table[(crc ^ buf[i]) & 0xFF] ^ (crc >> 8);
  }
  return crc ^ 0xFFFFFFFFu;
}

// ─── Little-endian helpers ──────────────────────────────────────────────────

void writeLE16(std::ostream& os, uint16_t v) {
  os.put(static_cast<char>(v & 0xFF));
  os.put(static_cast<char>((v >> 8) & 0xFF));
}

void writeLE32(std::ostream& os, uint32_t v) {
  os.put(static_cast<char>(v & 0xFF));
  os.put(static_cast<char>((v >> 8) & 0xFF));
  os.put(static_cast<char>((v >> 16) & 0xFF));
  os.put(static_cast<char>((v >> 24) & 0xFF));
}

uint16_t readLE16(std::istream& is) {
  uint8_t b[2];
  is.read(reinterpret_cast<char*>(b), 2);
  return static_cast<uint16_t>(b[0]) | (static_cast<uint16_t>(b[1]) << 8);
}

uint32_t readLE32(std::istream& is) {
  uint8_t b[4];
  is.read(reinterpret_cast<char*>(b), 4);
  return static_cast<uint32_t>(b[0]) | (static_cast<uint32_t>(b[1]) << 8) |
         (static_cast<uint32_t>(b[2]) << 16) |
         (static_cast<uint32_t>(b[3]) << 24);
}

// ─── ZIP STORE writer ───────────────────────────────────────────────────────

struct ZipEntry {
  std::string name;
  uint32_t crc;
  uint32_t size;
  uint32_t offset;  // offset of local file header
};

void writeLocalHeader(std::ostream& os, const ZipEntry& e) {
  writeLE32(os, 0x04034b50);                                 // signature
  writeLE16(os, 20);                                          // version needed
  writeLE16(os, 0);                                           // flags
  writeLE16(os, 0);                                           // compression: STORE
  writeLE16(os, 0);                                           // mod time
  writeLE16(os, 0);                                           // mod date
  writeLE32(os, e.crc);                                       // CRC-32
  writeLE32(os, e.size);                                      // compressed size
  writeLE32(os, e.size);                                      // uncompressed size
  writeLE16(os, static_cast<uint16_t>(e.name.size()));        // filename length
  writeLE16(os, 0);                                           // extra field length
  os.write(e.name.data(), e.name.size());                     // filename
}

void writeCentralHeader(std::ostream& os, const ZipEntry& e) {
  writeLE32(os, 0x02014b50);                                  // signature
  writeLE16(os, 20);                                          // version made by
  writeLE16(os, 20);                                          // version needed
  writeLE16(os, 0);                                           // flags
  writeLE16(os, 0);                                           // compression: STORE
  writeLE16(os, 0);                                           // mod time
  writeLE16(os, 0);                                           // mod date
  writeLE32(os, e.crc);                                       // CRC-32
  writeLE32(os, e.size);                                      // compressed size
  writeLE32(os, e.size);                                      // uncompressed size
  writeLE16(os, static_cast<uint16_t>(e.name.size()));        // filename length
  writeLE16(os, 0);                                           // extra field length
  writeLE16(os, 0);                                           // comment length
  writeLE16(os, 0);                                           // disk number
  writeLE16(os, 0);                                           // internal attrs
  writeLE32(os, 0);                                           // external attrs
  writeLE32(os, e.offset);                                    // local header offset
  os.write(e.name.data(), e.name.size());                     // filename
}

void writeEndOfCentralDir(std::ostream& os, uint16_t count,
                          uint32_t cd_size, uint32_t cd_offset) {
  writeLE32(os, 0x06054b50);   // signature
  writeLE16(os, 0);            // disk number
  writeLE16(os, 0);            // disk with central dir
  writeLE16(os, count);        // entries on this disk
  writeLE16(os, count);        // total entries
  writeLE32(os, cd_size);      // central dir size
  writeLE32(os, cd_offset);    // central dir offset
  writeLE16(os, 0);            // comment length
}

// ─── NumPy .npy format ──────────────────────────────────────────────────────

std::vector<char> buildNpyArray(const float* data, int rows, int cols) {
  // Eigen::MatrixXf is column-major = Fortran order
  std::ostringstream dict;
  dict << "{'descr': '<f4', 'fortran_order': True, 'shape': ("
       << rows << ", " << cols << "), }";
  std::string dict_str = dict.str();

  // Pad to 64-byte alignment
  size_t prefix_len = 10;  // 6 (magic) + 2 (version) + 2 (header_len)
  size_t padding = 64 - ((prefix_len + dict_str.size() + 1) % 64);
  if (padding == 64) padding = 0;
  dict_str.append(padding, ' ');
  dict_str.push_back('\n');

  uint16_t header_len = static_cast<uint16_t>(dict_str.size());
  size_t data_bytes = static_cast<size_t>(rows) * cols * sizeof(float);

  std::vector<char> buf;
  buf.reserve(prefix_len + header_len + data_bytes);

  // Magic + version
  const char magic[] = {'\x93', 'N', 'U', 'M', 'P', 'Y', '\x01', '\x00'};
  buf.insert(buf.end(), magic, magic + 8);

  // Header length (little-endian)
  buf.push_back(static_cast<char>(header_len & 0xFF));
  buf.push_back(static_cast<char>((header_len >> 8) & 0xFF));

  // Header dict + data
  buf.insert(buf.end(), dict_str.begin(), dict_str.end());
  const char* raw = reinterpret_cast<const char*>(data);
  buf.insert(buf.end(), raw, raw + data_bytes);

  return buf;
}

std::vector<char> buildNpyString(const std::string& str) {
  std::ostringstream dict;
  dict << "{'descr': '|S" << str.size()
       << "', 'fortran_order': False, 'shape': (), }";
  std::string dict_str = dict.str();

  size_t prefix_len = 10;
  size_t padding = 64 - ((prefix_len + dict_str.size() + 1) % 64);
  if (padding == 64) padding = 0;
  dict_str.append(padding, ' ');
  dict_str.push_back('\n');

  uint16_t header_len = static_cast<uint16_t>(dict_str.size());

  std::vector<char> buf;
  buf.reserve(prefix_len + header_len + str.size());

  const char magic[] = {'\x93', 'N', 'U', 'M', 'P', 'Y', '\x01', '\x00'};
  buf.insert(buf.end(), magic, magic + 8);
  buf.push_back(static_cast<char>(header_len & 0xFF));
  buf.push_back(static_cast<char>((header_len >> 8) & 0xFF));

  buf.insert(buf.end(), dict_str.begin(), dict_str.end());
  buf.insert(buf.end(), str.begin(), str.end());

  return buf;
}

std::string escapeJson(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    if (c == '"')
      out += "\\\"";
    else if (c == '\\')
      out += "\\\\";
    else
      out += c;
  }
  return out;
}

constexpr int kMetadataVersion = 1;

std::string buildMetadataJson(const ElevationMap& map) {
  auto pos = map.getPosition();
  auto size = map.getSize();
  auto start = map.getStartIndex();

  std::ostringstream json;
  json << "{\"version\": " << kMetadataVersion
       << ", \"resolution\": " << map.getResolution()
       << ", \"position\": [" << pos.x() << ", " << pos.y() << "]"
       << ", \"frame_id\": \"" << escapeJson(map.getFrameId()) << "\""
       << ", \"size\": [" << size(0) << ", " << size(1) << "]"
       << ", \"start_index\": [" << start(0) << ", " << start(1) << "]"
       << "}";
  return json.str();
}

// ─── Metadata JSON parser (minimal) ────────────────────────────────────────

// Extract a numeric value after "key": from JSON string.
bool jsonFloat(const std::string& json, const std::string& key, float& out) {
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return false;
  pos = json.find(':', pos);
  if (pos == std::string::npos) return false;
  try {
    out = std::stof(json.substr(pos + 1));
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

// Extract two ints from "key": [a, b].
bool jsonIntPair(const std::string& json, const std::string& key, int& a,
                 int& b) {
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return false;
  pos = json.find('[', pos);
  if (pos == std::string::npos) return false;
  auto end = json.find(']', pos);
  if (end == std::string::npos) return false;
  std::string inner = json.substr(pos + 1, end - pos - 1);
  auto comma = inner.find(',');
  if (comma == std::string::npos) return false;
  try {
    a = std::stoi(inner.substr(0, comma));
    b = std::stoi(inner.substr(comma + 1));
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

// Extract two floats from "key": [a, b].
bool jsonFloatPair(const std::string& json, const std::string& key, float& a,
                   float& b) {
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return false;
  pos = json.find('[', pos);
  if (pos == std::string::npos) return false;
  auto end = json.find(']', pos);
  if (end == std::string::npos) return false;
  std::string inner = json.substr(pos + 1, end - pos - 1);
  auto comma = inner.find(',');
  if (comma == std::string::npos) return false;
  try {
    a = std::stof(inner.substr(0, comma));
    b = std::stof(inner.substr(comma + 1));
  } catch (const std::exception&) {
    return false;
  }
  return true;
}

// Extract a quoted string value after "key": "value".
bool jsonString(const std::string& json, const std::string& key,
                std::string& out) {
  auto pos = json.find("\"" + key + "\"");
  if (pos == std::string::npos) return false;
  pos = json.find(':', pos);
  if (pos == std::string::npos) return false;
  auto q1 = json.find('"', pos + 1);
  if (q1 == std::string::npos) return false;
  auto q2 = json.find('"', q1 + 1);
  if (q2 == std::string::npos) return false;
  out = json.substr(q1 + 1, q2 - q1 - 1);
  return true;
}

// ─── .npy header parser ────────────────────────────────────────────────────

struct NpyInfo {
  int rows = 0;
  int cols = 0;
  bool is_float_array = false;  // '<f4' with shape (r,c)
  bool is_string = false;       // '|S...' with shape ()
  size_t string_len = 0;
  size_t data_offset = 0;  // offset to data within .npy blob
};

bool parseNpyHeader(const char* buf, size_t buf_size, NpyInfo& info) {
  // Validate magic: \x93NUMPY
  if (buf_size < 10) return false;
  if (buf[0] != '\x93' || buf[1] != 'N' || buf[2] != 'U' || buf[3] != 'M' ||
      buf[4] != 'P' || buf[5] != 'Y') {
    return false;
  }

  uint16_t header_len = static_cast<uint8_t>(buf[8]) |
                         (static_cast<uint16_t>(static_cast<uint8_t>(buf[9]))
                          << 8);
  info.data_offset = 10 + header_len;
  if (info.data_offset > buf_size) return false;

  std::string dict(buf + 10, header_len);

  // Check descr
  auto descr_pos = dict.find("'descr'");
  if (descr_pos == std::string::npos) return false;

  if (dict.find("'<f4'") != std::string::npos) {
    info.is_float_array = true;
    // Parse shape: (rows, cols)
    auto shape_pos = dict.find("'shape'");
    if (shape_pos == std::string::npos) return false;
    auto paren = dict.find('(', shape_pos);
    if (paren == std::string::npos) return false;
    auto paren_end = dict.find(')', paren);
    if (paren_end == std::string::npos) return false;
    std::string shape_str = dict.substr(paren + 1, paren_end - paren - 1);
    auto comma = shape_str.find(',');
    if (comma == std::string::npos) return false;
    info.rows = std::stoi(shape_str.substr(0, comma));
    std::string cols_str = shape_str.substr(comma + 1);
    // Handle trailing comma: "(200, 100, )"
    if (!cols_str.empty() && cols_str.back() == ',') cols_str.pop_back();
    while (!cols_str.empty() && cols_str.front() == ' ') cols_str.erase(0, 1);
    if (cols_str.empty()) return false;
    info.cols = std::stoi(cols_str);
  } else if (dict.find("'|S") != std::string::npos) {
    info.is_string = true;
    auto s_pos = dict.find("'|S");
    auto s_end = dict.find('\'', s_pos + 3);
    if (s_end == std::string::npos) return false;
    info.string_len = std::stoul(dict.substr(s_pos + 3, s_end - s_pos - 3));
  } else {
    return false;
  }

  return true;
}

}  // namespace detail

// ─── Save ───────────────────────────────────────────────────────────────────

bool saveNpz(const std::string& filename, const ElevationMap& map) {
  return saveNpz(filename, map, map.getLayers());
}

bool saveNpz(const std::string& filename, const ElevationMap& map,
             const std::vector<std::string>& layer_names) {
  std::ofstream fs(filename, std::ios::binary);
  if (!fs.is_open()) {
    spdlog::error("[npz_io] Cannot create {}", filename);
    return false;
  }

  std::vector<detail::ZipEntry> entries;
  int rows = map.getSize()(0);
  int cols = map.getSize()(1);

  // Helper: add a .npy blob to the archive
  auto addEntry = [&](const std::string& name, const std::vector<char>& npy) {
    detail::ZipEntry e;
    e.name = name;
    e.crc = detail::crc32(npy.data(), npy.size());
    e.size = static_cast<uint32_t>(npy.size());
    e.offset = static_cast<uint32_t>(fs.tellp());
    detail::writeLocalHeader(fs, e);
    fs.write(npy.data(), npy.size());
    entries.push_back(std::move(e));
  };

  // Write each layer
  for (const auto& name : layer_names) {
    if (!map.exists(name)) {
      spdlog::warn("[npz_io] Layer '{}' does not exist, skipping", name);
      continue;
    }
    const auto& matrix = map.get(name);
    addEntry(name + ".npy", detail::buildNpyArray(matrix.data(), rows, cols));
  }

  // Write metadata
  std::string meta_json = detail::buildMetadataJson(map);
  addEntry("meta.npy", detail::buildNpyString(meta_json));

  // Central directory
  uint32_t cd_offset = static_cast<uint32_t>(fs.tellp());
  for (const auto& e : entries) {
    detail::writeCentralHeader(fs, e);
  }
  uint32_t cd_size = static_cast<uint32_t>(fs.tellp()) - cd_offset;

  // End of central directory
  detail::writeEndOfCentralDir(fs, static_cast<uint16_t>(entries.size()),
                               cd_size, cd_offset);

  if (fs.fail()) {
    spdlog::error("[npz_io] Write failed for {}", filename);
    return false;
  }

  return true;
}

// ─── Load ───────────────────────────────────────────────────────────────────

bool loadNpz(const std::string& filename, ElevationMap& map) {
  std::ifstream fs(filename, std::ios::binary);
  if (!fs.is_open()) {
    spdlog::error("[npz_io] Cannot open {}", filename);
    return false;
  }

  // Collect entries by reading local file headers sequentially
  struct Entry {
    std::string name;
    std::vector<char> data;
  };
  std::vector<Entry> entries;
  std::string metadata_json;

  constexpr uint32_t kLocalSig = 0x04034b50;
  constexpr uint32_t kMaxEntries = 1000;
  constexpr uint32_t kMaxEntrySize = 400'000'000;  // 400MB

  while (entries.size() < kMaxEntries) {
    uint32_t sig = detail::readLE32(fs);
    if (fs.fail() || sig != kLocalSig) break;

    // Skip: version(2), flags(2), compression(2), time(2), date(2), crc(4)
    fs.ignore(14);
    // compressed_size == uncompressed_size (STORE)
    /*uint32_t compressed_size =*/detail::readLE32(fs);
    uint32_t uncompressed_size = detail::readLE32(fs);
    uint16_t name_len = detail::readLE16(fs);
    uint16_t extra_len = detail::readLE16(fs);
    if (fs.fail()) break;

    // Validate
    if (name_len > 4096 || uncompressed_size > kMaxEntrySize) {
      spdlog::error("[npz_io] Invalid entry (name_len={}, size={})", name_len,
                    uncompressed_size);
      return false;
    }

    // Read filename
    std::string name(name_len, '\0');
    fs.read(&name[0], name_len);
    fs.ignore(extra_len);

    // Read data
    std::vector<char> data(uncompressed_size);
    fs.read(data.data(), uncompressed_size);
    if (fs.fail()) {
      spdlog::error("[npz_io] Truncated data for entry '{}'", name);
      return false;
    }

    entries.push_back({std::move(name), std::move(data)});
  }

  if (entries.empty()) {
    spdlog::error("[npz_io] No entries found in {}", filename);
    return false;
  }

  // Find and parse metadata entry
  bool found_meta = false;
  for (const auto& e : entries) {
    if (e.name == "meta.npy") {
      detail::NpyInfo info;
      if (!detail::parseNpyHeader(e.data.data(), e.data.size(), info) ||
          !info.is_string) {
        spdlog::error("[npz_io] Invalid meta.npy header");
        return false;
      }
      if (info.data_offset + info.string_len > e.data.size()) {
        spdlog::error("[npz_io] meta.npy data truncated");
        return false;
      }
      metadata_json.assign(e.data.data() + info.data_offset, info.string_len);
      found_meta = true;
      break;
    }
  }

  if (!found_meta) {
    spdlog::error("[npz_io] No meta.npy entry in {}", filename);
    return false;
  }

  // Parse metadata
  float version = 0;
  float resolution = 0;
  float pos_x = 0, pos_y = 0;
  int size_rows = 0, size_cols = 0;
  int start_x = 0, start_y = 0;
  std::string frame_id;

  // Version check (absent in pre-v1 files → allow gracefully)
  if (detail::jsonFloat(metadata_json, "version", version)) {
    if (static_cast<int>(version) > detail::kMetadataVersion) {
      spdlog::error("[npz_io] Unsupported metadata version {} (max {})",
                    static_cast<int>(version), detail::kMetadataVersion);
      return false;
    }
  }

  if (!detail::jsonFloat(metadata_json, "resolution", resolution) ||
      !detail::jsonFloatPair(metadata_json, "position", pos_x, pos_y) ||
      !detail::jsonIntPair(metadata_json, "size", size_rows, size_cols)) {
    spdlog::error("[npz_io] Incomplete metadata in {}", filename);
    return false;
  }
  detail::jsonString(metadata_json, "frame_id", frame_id);
  detail::jsonIntPair(metadata_json, "start_index", start_x, start_y);

  // Validate dimensions
  if (size_rows <= 0 || size_cols <= 0 || resolution <= 0) {
    spdlog::error("[npz_io] Invalid map dimensions ({}x{}, res={})", size_rows,
                  size_cols, resolution);
    return false;
  }

  // Initialize map geometry
  float length_x = resolution * size_rows;
  float length_y = resolution * size_cols;
  map.setFrameId(frame_id);
  map.setGeometry(length_x, length_y, resolution);
  map.setPosition(grid_map::Position(pos_x, pos_y));
  map.setStartIndex(grid_map::Index(start_x, start_y));

  // Load layer data
  size_t expected_bytes =
      static_cast<size_t>(size_rows) * size_cols * sizeof(float);
  int layers_loaded = 0;

  for (const auto& e : entries) {
    if (e.name == "meta.npy") continue;

    // Strip .npy suffix to get layer name
    if (e.name.size() <= 4 || e.name.substr(e.name.size() - 4) != ".npy") {
      continue;
    }
    std::string layer_name = e.name.substr(0, e.name.size() - 4);

    detail::NpyInfo info;
    if (!detail::parseNpyHeader(e.data.data(), e.data.size(), info) ||
        !info.is_float_array) {
      spdlog::warn("[npz_io] Skipping non-float entry '{}'", e.name);
      continue;
    }

    if (info.rows != size_rows || info.cols != size_cols) {
      spdlog::warn("[npz_io] Shape mismatch for '{}': ({}x{}) vs ({}x{})",
                   layer_name, info.rows, info.cols, size_rows, size_cols);
      continue;
    }

    if (info.data_offset + expected_bytes > e.data.size()) {
      spdlog::warn("[npz_io] Truncated data for '{}'", layer_name);
      continue;
    }

    // Add layer and copy data (column-major, matching Eigen layout)
    if (!map.exists(layer_name)) {
      map.add(layer_name);
    }
    auto& matrix = map.get(layer_name);
    std::memcpy(matrix.data(), e.data.data() + info.data_offset,
                expected_bytes);
    ++layers_loaded;
  }

  if (layers_loaded == 0) {
    spdlog::error("[npz_io] No layer data found in {}", filename);
    return false;
  }

  return true;
}

}  // namespace io
}  // namespace fastdem
