/*
 * layers.h
 *
 * Layer definitions for height_mapping_core
 */

#ifndef HEIGHT_MAPPING_CORE_LAYERS_H
#define HEIGHT_MAPPING_CORE_LAYERS_H

namespace height_mapping {
namespace layers {

// Core height layers
namespace Height {
static constexpr const char *ELEVATION = "elevation";
static constexpr const char *MIN = "height_min";
static constexpr const char *MAX = "height_max";
static constexpr const char *VARIANCE = "height_variance";
static constexpr const char *MEASUREMENT_COUNT = "measurement_count";
} // namespace Height

// Confidence layers
namespace Confidence {
static constexpr const char *STANDARD_ERROR = "standard_error";
static constexpr const char *CONFIDENCE_INTERVAL = "confidence_interval";
static constexpr const char *CONFIDENCE = "confidence";
} // namespace Confidence

// Sensor data layers
namespace Sensor {
namespace Lidar {
static constexpr const char *INTENSITY = "intensity";
}

namespace RGBD {
static constexpr const char *R = "r";
static constexpr const char *G = "g";
static constexpr const char *B = "b";
static constexpr const char *COLOR = "color";
} // namespace RGBD
} // namespace Sensor

// Scan processing layers
namespace Scan {
static constexpr const char *RAY_CASTING = "ray_casting";
static constexpr const char *SCAN_HEIGHT = "scan_height";
} // namespace Scan

} // namespace layers
} // namespace height_mapping

#endif // HEIGHT_MAPPING_CORE_LAYERS_H