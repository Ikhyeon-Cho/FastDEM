// nanoPCL - Header-only C++17 point cloud library
// SPDX-License-Identifier: MIT
//
// PCD (Point Cloud Data) file I/O.
//
// Supports:
//   - ASCII and BINARY formats
//   - Automatic channel detection (intensity, rgb, normal)
//   - VIEWPOINT metadata preservation
//
// Example:
//   auto cloud = io::loadPCD("scan.pcd");
//   io::savePCD("output.pcd", cloud, {.format = PCDFormat::BINARY});

#ifndef NANOPCL_IO_PCD_IO_HPP
#define NANOPCL_IO_PCD_IO_HPP

#include <Eigen/Geometry>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "nanopcl/core/point_cloud.hpp"

namespace nanopcl {
namespace io {

// ===========
// Exceptions
// ===========

class IOException : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

// =====================
// Enums and Structures
// =====================

enum class PCDFormat { ASCII,
                       BINARY };

/// @brief Metadata from PCD file header
struct PCDMetadata {
  Eigen::Isometry3d viewpoint = Eigen::Isometry3d::Identity();
  uint32_t width = 0;
  uint32_t height = 1;
  uint32_t num_points = 0;
};

/// @brief Options for saving PCD files
struct PCDSaveOptions {
  PCDFormat format = PCDFormat::BINARY;
  Eigen::Isometry3d viewpoint = Eigen::Isometry3d::Identity();
  int precision = 8; // For ASCII format
};

// =========================
// Internal: Header Parsing
// =========================

namespace detail {

struct PCDFieldInfo {
  std::string name;
  char type;       // F (float), U (unsigned), I (signed)
  uint32_t size;   // bytes per element
  uint32_t count;  // elements per field
  uint32_t offset; // byte offset in point record
};

struct PCDHeader {
  std::vector<PCDFieldInfo> fields;
  uint32_t width = 0;
  uint32_t height = 1;
  uint32_t point_size = 0; // total bytes per point
  Eigen::Isometry3d viewpoint = Eigen::Isometry3d::Identity();
  PCDFormat format = PCDFormat::ASCII;

  [[nodiscard]] uint32_t numPoints() const { return width * height; }

  [[nodiscard]] int findField(const std::string& name) const {
    for (size_t i = 0; i < fields.size(); ++i) {
      if (fields[i].name == name) return static_cast<int>(i);
    }
    return -1;
  }
};

inline std::string toLower(const std::string& s) {
  std::string result = s;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) { return std::tolower(c); });
  return result;
}

inline std::vector<std::string> split(const std::string& s) {
  std::vector<std::string> tokens;
  std::istringstream iss(s);
  std::string token;
  while (iss >> token) {
    tokens.push_back(token);
  }
  return tokens;
}

inline PCDHeader parseHeader(std::istream& is) {
  PCDHeader header;
  std::string line;

  std::vector<std::string> field_names;
  std::vector<uint32_t> field_sizes;
  std::vector<char> field_types;
  std::vector<uint32_t> field_counts;

  while (std::getline(is, line)) {
    if (line.empty() || line[0] == '#') continue;

    auto tokens = split(line);
    if (tokens.empty()) continue;

    std::string key = toLower(tokens[0]);

    if (key == "version") {
      // PCD version (ignored)
    } else if (key == "fields") {
      for (size_t i = 1; i < tokens.size(); ++i) {
        field_names.push_back(toLower(tokens[i]));
      }
    } else if (key == "size") {
      for (size_t i = 1; i < tokens.size(); ++i) {
        field_sizes.push_back(std::stoul(tokens[i]));
      }
    } else if (key == "type") {
      for (size_t i = 1; i < tokens.size(); ++i) {
        field_types.push_back(tokens[i][0]);
      }
    } else if (key == "count") {
      for (size_t i = 1; i < tokens.size(); ++i) {
        field_counts.push_back(std::stoul(tokens[i]));
      }
    } else if (key == "width") {
      header.width = std::stoul(tokens[1]);
    } else if (key == "height") {
      header.height = std::stoul(tokens[1]);
    } else if (key == "viewpoint") {
      if (tokens.size() >= 8) {
        double tx = std::stod(tokens[1]);
        double ty = std::stod(tokens[2]);
        double tz = std::stod(tokens[3]);
        double qw = std::stod(tokens[4]);
        double qx = std::stod(tokens[5]);
        double qy = std::stod(tokens[6]);
        double qz = std::stod(tokens[7]);
        header.viewpoint.translation() = Eigen::Vector3d(tx, ty, tz);
        header.viewpoint.linear() =
            Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
      }
    } else if (key == "points") {
      // Explicit point count (may differ from width*height)
    } else if (key == "data") {
      if (tokens.size() >= 2) {
        std::string fmt = toLower(tokens[1]);
        if (fmt == "ascii") {
          header.format = PCDFormat::ASCII;
        } else if (fmt == "binary") {
          header.format = PCDFormat::BINARY;
        } else if (fmt == "binary_compressed") {
          throw IOException("PCD binary_compressed format not supported");
        }
      }
      break; // DATA is the last header line
    }
  }

  // Build field info with offsets
  if (field_names.empty()) {
    throw IOException("PCD header missing FIELDS");
  }

  // Default counts to 1 if not specified
  if (field_counts.empty()) {
    field_counts.resize(field_names.size(), 1);
  }

  uint32_t offset = 0;
  for (size_t i = 0; i < field_names.size(); ++i) {
    PCDFieldInfo info;
    info.name = field_names[i];
    info.size = (i < field_sizes.size()) ? field_sizes[i] : 4;
    info.type = (i < field_types.size()) ? field_types[i] : 'F';
    info.count = (i < field_counts.size()) ? field_counts[i] : 1;
    info.offset = offset;
    offset += info.size * info.count;
    header.fields.push_back(info);
  }
  header.point_size = offset;

  return header;
}

inline float readFieldAsFloat(const char* data, const PCDFieldInfo& field) {
  if (field.type == 'F' && field.size == 4) {
    float v;
    std::memcpy(&v, data, 4);
    return v;
  } else if (field.type == 'F' && field.size == 8) {
    double v;
    std::memcpy(&v, data, 8);
    return static_cast<float>(v);
  } else if (field.type == 'U' && field.size == 1) {
    return static_cast<float>(*reinterpret_cast<const uint8_t*>(data));
  } else if (field.type == 'U' && field.size == 4) {
    uint32_t v;
    std::memcpy(&v, data, 4);
    return static_cast<float>(v);
  } else if (field.type == 'I' && field.size == 4) {
    int32_t v;
    std::memcpy(&v, data, 4);
    return static_cast<float>(v);
  }
  return 0.0f;
}

} // namespace detail

// ===============
// Load Functions
// ===============

/// @brief Load PCD from stream (core implementation)
/// @param is Input stream
/// @param meta_out Output metadata
/// @return Loaded point cloud
/// @throws IOException on parse error
inline PointCloud loadPCD(std::istream& is, PCDMetadata& meta_out) {
  if (!is) {
    throw IOException("Invalid input stream");
  }

  auto header = detail::parseHeader(is);

  meta_out.width = header.width;
  meta_out.height = header.height;
  meta_out.num_points = header.numPoints();
  meta_out.viewpoint = header.viewpoint;

  if (meta_out.num_points == 0) {
    return PointCloud();
  }

  // Find field indices
  int idx_x = header.findField("x");
  int idx_y = header.findField("y");
  int idx_z = header.findField("z");

  if (idx_x < 0 || idx_y < 0 || idx_z < 0) {
    throw IOException("PCD file missing x, y, z fields");
  }

  // Optional fields
  int idx_intensity = header.findField("intensity");
  if (idx_intensity < 0) idx_intensity = header.findField("i");
  if (idx_intensity < 0) idx_intensity = header.findField("reflectivity");

  int idx_rgb = header.findField("rgb");
  if (idx_rgb < 0) idx_rgb = header.findField("rgba");

  int idx_nx = header.findField("normal_x");
  int idx_ny = header.findField("normal_y");
  int idx_nz = header.findField("normal_z");
  if (idx_nx < 0) idx_nx = header.findField("nx");
  if (idx_ny < 0) idx_ny = header.findField("ny");
  if (idx_nz < 0) idx_nz = header.findField("nz");

  // Prepare cloud with channels
  PointCloud cloud;
  cloud.reserve(meta_out.num_points);

  bool has_intensity = (idx_intensity >= 0);
  bool has_rgb = (idx_rgb >= 0);
  bool has_normal = (idx_nx >= 0 && idx_ny >= 0 && idx_nz >= 0);

  if (has_intensity) cloud.useIntensity();
  if (has_rgb) cloud.useColor();
  if (has_normal) cloud.useNormal();

  // Read data
  if (header.format == PCDFormat::ASCII) {
    std::string line;
    for (uint32_t i = 0; i < meta_out.num_points; ++i) {
      if (!std::getline(is, line)) {
        throw IOException("Unexpected end of ASCII data");
      }

      auto tokens = detail::split(line);
      if (tokens.size() < header.fields.size()) {
        throw IOException("Incomplete point data at line " + std::to_string(i));
      }

      float x = std::stof(tokens[idx_x]);
      float y = std::stof(tokens[idx_y]);
      float z = std::stof(tokens[idx_z]);

      cloud.add(x, y, z);

      if (has_intensity) {
        cloud.intensities().back() = std::stof(tokens[idx_intensity]);
      }
      if (has_rgb) {
        // RGB is packed as uint32 or float
        uint32_t rgb_packed = std::stoul(tokens[idx_rgb]);
        uint8_t r = (rgb_packed >> 16) & 0xFF;
        uint8_t g = (rgb_packed >> 8) & 0xFF;
        uint8_t b = rgb_packed & 0xFF;
        cloud.colors().back() = Color(r, g, b);
      }
      if (has_normal) {
        float nx = std::stof(tokens[idx_nx]);
        float ny = std::stof(tokens[idx_ny]);
        float nz = std::stof(tokens[idx_nz]);
        cloud.normals().back() = Normal4(nx, ny, nz, 0.0f);
      }
    }
  } else {
    // Binary format
    std::vector<char> buffer(header.point_size);

    for (uint32_t i = 0; i < meta_out.num_points; ++i) {
      if (!is.read(buffer.data(), header.point_size)) {
        throw IOException("Unexpected end of binary data");
      }

      const auto& fx = header.fields[idx_x];
      const auto& fy = header.fields[idx_y];
      const auto& fz = header.fields[idx_z];

      float x = detail::readFieldAsFloat(buffer.data() + fx.offset, fx);
      float y = detail::readFieldAsFloat(buffer.data() + fy.offset, fy);
      float z = detail::readFieldAsFloat(buffer.data() + fz.offset, fz);

      cloud.add(x, y, z);

      if (has_intensity) {
        const auto& fi = header.fields[idx_intensity];
        cloud.intensities().back() =
            detail::readFieldAsFloat(buffer.data() + fi.offset, fi);
      }
      if (has_rgb) {
        const auto& fc = header.fields[idx_rgb];
        uint32_t rgb_packed;
        std::memcpy(&rgb_packed, buffer.data() + fc.offset, 4);
        uint8_t r = (rgb_packed >> 16) & 0xFF;
        uint8_t g = (rgb_packed >> 8) & 0xFF;
        uint8_t b = rgb_packed & 0xFF;
        cloud.colors().back() = Color(r, g, b);
      }
      if (has_normal) {
        const auto& fnx = header.fields[idx_nx];
        const auto& fny = header.fields[idx_ny];
        const auto& fnz = header.fields[idx_nz];
        float nx = detail::readFieldAsFloat(buffer.data() + fnx.offset, fnx);
        float ny = detail::readFieldAsFloat(buffer.data() + fny.offset, fny);
        float nz = detail::readFieldAsFloat(buffer.data() + fnz.offset, fnz);
        cloud.normals().back() = Normal4(nx, ny, nz, 0.0f);
      }
    }
  }

  return cloud;
}

/// @brief Load PCD from file (simple version)
/// @param path File path
/// @return Loaded point cloud
/// @throws IOException on file/parse error
inline PointCloud loadPCD(const std::string& path) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    throw IOException("Cannot open file: " + path);
  }
  PCDMetadata meta;
  return loadPCD(ifs, meta);
}

/// @brief Load PCD from file with metadata
/// @param path File path
/// @param meta_out Output metadata
/// @return Loaded point cloud
/// @throws IOException on file/parse error
inline PointCloud loadPCD(const std::string& path, PCDMetadata& meta_out) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    throw IOException("Cannot open file: " + path);
  }
  return loadPCD(ifs, meta_out);
}

// ===============
// Save Functions
// ===============

/// @brief Save PCD to stream (core implementation)
/// @param os Output stream
/// @param cloud Point cloud to save
/// @param options Save options
/// @throws IOException on write error
inline void savePCD(std::ostream& os,
                    const PointCloud& cloud,
                    const PCDSaveOptions& options) {
  if (!os) {
    throw IOException("Invalid output stream");
  }

  const size_t n = cloud.size();
  const bool has_intensity = cloud.hasIntensity();
  const bool has_rgb = cloud.hasColor();
  const bool has_normal = cloud.hasNormal();

  // Build field list
  std::vector<std::string> field_names = {"x", "y", "z"};
  std::vector<uint32_t> field_sizes = {4, 4, 4};
  std::vector<char> field_types = {'F', 'F', 'F'};

  if (has_intensity) {
    field_names.push_back("intensity");
    field_sizes.push_back(4);
    field_types.push_back('F');
  }
  if (has_rgb) {
    field_names.push_back("rgb");
    field_sizes.push_back(4);
    field_types.push_back('U');
  }
  if (has_normal) {
    field_names.push_back("normal_x");
    field_names.push_back("normal_y");
    field_names.push_back("normal_z");
    field_sizes.push_back(4);
    field_sizes.push_back(4);
    field_sizes.push_back(4);
    field_types.push_back('F');
    field_types.push_back('F');
    field_types.push_back('F');
  }

  // Write header
  os << "# .PCD v0.7 - Point Cloud Data file format\n";
  os << "VERSION 0.7\n";

  os << "FIELDS";
  for (const auto& name : field_names)
    os << " " << name;
  os << "\n";

  os << "SIZE";
  for (auto sz : field_sizes)
    os << " " << sz;
  os << "\n";

  os << "TYPE";
  for (auto t : field_types)
    os << " " << t;
  os << "\n";

  os << "COUNT";
  for (size_t i = 0; i < field_names.size(); ++i)
    os << " 1";
  os << "\n";

  os << "WIDTH " << n << "\n";
  os << "HEIGHT 1\n";

  // Viewpoint
  const auto& vp = options.viewpoint;
  Eigen::Quaterniond q(vp.rotation());
  os << "VIEWPOINT " << vp.translation().x() << " " << vp.translation().y()
     << " " << vp.translation().z() << " " << q.w() << " " << q.x() << " "
     << q.y() << " " << q.z() << "\n";

  os << "POINTS " << n << "\n";

  if (options.format == PCDFormat::ASCII) {
    os << "DATA ascii\n";
    os << std::fixed << std::setprecision(options.precision);

    for (size_t i = 0; i < n; ++i) {
      const auto& p = cloud.point(i);
      os << p.x() << " " << p.y() << " " << p.z();

      if (has_intensity) {
        os << " " << cloud.intensity(i);
      }
      if (has_rgb) {
        const auto& c = cloud.color(i);
        uint32_t rgb_packed =
            (static_cast<uint32_t>(c.r) << 16) |
            (static_cast<uint32_t>(c.g) << 8) |
            static_cast<uint32_t>(c.b);
        os << " " << rgb_packed;
      }
      if (has_normal) {
        const auto& nm = cloud.normal(i);
        os << " " << nm.x() << " " << nm.y() << " " << nm.z();
      }
      os << "\n";
    }
  } else {
    os << "DATA binary\n";

    for (size_t i = 0; i < n; ++i) {
      const auto& p = cloud.point(i);
      float x = p.x(), y = p.y(), z = p.z();
      os.write(reinterpret_cast<const char*>(&x), 4);
      os.write(reinterpret_cast<const char*>(&y), 4);
      os.write(reinterpret_cast<const char*>(&z), 4);

      if (has_intensity) {
        float intensity = cloud.intensity(i);
        os.write(reinterpret_cast<const char*>(&intensity), 4);
      }
      if (has_rgb) {
        const auto& c = cloud.color(i);
        uint32_t rgb_packed =
            (static_cast<uint32_t>(c.r) << 16) |
            (static_cast<uint32_t>(c.g) << 8) |
            static_cast<uint32_t>(c.b);
        os.write(reinterpret_cast<const char*>(&rgb_packed), 4);
      }
      if (has_normal) {
        const auto& nm = cloud.normal(i);
        float nx = nm.x(), ny = nm.y(), nz = nm.z();
        os.write(reinterpret_cast<const char*>(&nx), 4);
        os.write(reinterpret_cast<const char*>(&ny), 4);
        os.write(reinterpret_cast<const char*>(&nz), 4);
      }
    }
  }

  if (!os) {
    throw IOException("Error writing PCD data");
  }
}

/// @brief Save PCD to file
/// @param path File path
/// @param cloud Point cloud to save
/// @param options Save options
/// @throws IOException on file/write error
inline void savePCD(const std::string& path,
                    const PointCloud& cloud,
                    const PCDSaveOptions& options = PCDSaveOptions()) {
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs) {
    throw IOException("Cannot create file: " + path);
  }
  savePCD(ofs, cloud, options);
}

/// @brief Convenience overload with format parameter
inline void savePCD(const std::string& path,
                    const PointCloud& cloud,
                    PCDFormat format) {
  PCDSaveOptions options;
  options.format = format;
  savePCD(path, cloud, options);
}

} // namespace io
} // namespace nanopcl

#endif // NANOPCL_IO_PCD_IO_HPP
