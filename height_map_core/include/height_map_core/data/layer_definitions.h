#pragma once

namespace height_mapping {
namespace layers {

struct Height {
  static constexpr const char *ELEVATION = "elevation";
  static constexpr const char *ELEVATION_MIN = "elevation_min";
  static constexpr const char *ELEVATION_MAX = "elevation_max";
  static constexpr const char *ELEVATION_VARIANCE = "elevation_variance";
  static constexpr const char *N_MEASUREMENTS = "n_measured";
};

struct Scan {
  static constexpr const char *RAY_CASTING = "scan/raycasting";
  static constexpr const char *SCAN_HEIGHT = "scan/scan_height";
};

struct Confidence {
  static constexpr const char *STANDARD_ERROR = "height/standard_error";
  static constexpr const char *CONFIDENCE_INTERVAL = "height/confidence_interval";
  static constexpr const char *CONFIDENCE = "height/confidence";
};

struct Sensor {
  struct Lidar {
    static constexpr const char *INTENSITY = "lidar/intensity";
  };

  struct RGBD {
    static constexpr const char *R = "rgbd/r";
    static constexpr const char *G = "rgbd/g";
    static constexpr const char *B = "rgbd/b";
    static constexpr const char *COLOR = "rgbd/color";
  };
};

} // namespace layers
} // namespace height_mapping
