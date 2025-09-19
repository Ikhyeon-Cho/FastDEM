/*
 * height_estimation_stage.h
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef HEIGHT_MAPPING_CORE_PIPELINE_STAGES_HEIGHT_ESTIMATION_STAGE_H
#define HEIGHT_MAPPING_CORE_PIPELINE_STAGES_HEIGHT_ESTIMATION_STAGE_H

#include "height_mapping_core/layers.h"
#include "height_mapping_core/pipeline/processing_context.h"
#include "height_mapping_core/pipeline/stage.h"
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace height_mapping::core {

// Interface for height estimators
class IHeightEstimator {
public:
  virtual ~IHeightEstimator() = default;

  // Update height and variance with new measurement
  virtual void update(float &height, float &variance, float measurement) = 0;

  // Reset estimator state
  virtual void reset() {}

  // Get estimator type name
  virtual std::string getType() const = 0;
};

// Simple averaging estimator
class MeanEstimator : public IHeightEstimator {
public:
  void update(float &height, float &variance, float measurement) override {
    if (!std::isfinite(height)) {
      // First measurement
      height = measurement;
      variance = initial_variance_;
      return;
    }

    // Simple averaging
    float alpha = 0.5f; // Weight for new measurement
    height = alpha * measurement + (1 - alpha) * height;
    variance = variance * 0.95f; // Decrease variance with each measurement
  }

  std::string getType() const override { return "Mean"; }

private:
  float initial_variance_ = 1.0f;
};

// Kalman filter estimator
class KalmanEstimator : public IHeightEstimator {
public:
  KalmanEstimator(float process_noise = 0.01f, float measurement_noise = 0.1f)
      : process_noise_(process_noise), measurement_noise_(measurement_noise) {}

  void update(float &height, float &variance, float measurement) override {
    if (!std::isfinite(height)) {
      // Initialize
      height = measurement;
      variance = measurement_noise_;
      return;
    }

    // Kalman filter update
    // Prediction
    float predicted_variance = variance + process_noise_;

    // Update
    float kalman_gain =
        predicted_variance / (predicted_variance + measurement_noise_);
    height = height + kalman_gain * (measurement - height);
    variance = (1 - kalman_gain) * predicted_variance;
  }

  std::string getType() const override { return "Kalman"; }

private:
  float process_noise_;
  float measurement_noise_;
};

// Simple averaging estimator - stateless version
// Uses exponential moving average for simplicity
class IncrementalMeanEstimator : public IHeightEstimator {
public:
  void update(float &height, float &variance, float measurement) override {
    if (!std::isfinite(height)) {
      // First measurement
      height = measurement;
      variance = 0.01f; // Initial uncertainty
      return;
    }

    // Simple exponential moving average
    // Weight decreases over time to converge to stable value
    float alpha =
        1.0f /
        (2.0f + variance * 100.0f); // Adaptive weight based on current variance
    alpha = std::max(0.1f, std::min(0.9f, alpha)); // Clamp to reasonable range

    float prev_height = height;
    height = alpha * measurement + (1.0f - alpha) * prev_height;

    // Update variance (decreases with more measurements)
    float measurement_variance =
        (measurement - height) * (measurement - height);
    variance = alpha * measurement_variance + (1.0f - alpha) * variance * 0.95f;
  }

  void reset() override {
    // Stateless - nothing to reset
  }

  std::string getType() const override { return "IncrementalMean"; }
};

// Moving Average Estimator - stateless version
// Approximates moving average with exponential weighting
class MovingAverageEstimator : public IHeightEstimator {
public:
  explicit MovingAverageEstimator(size_t window_size = 10)
      : window_size_(window_size) {}

  void update(float &height, float &variance, float measurement) override {
    if (!std::isfinite(height)) {
      // First measurement
      height = measurement;
      variance = 0.01f;
      return;
    }

    // Exponential moving average with window size influence
    float alpha = 2.0f / (window_size_ + 1.0f);

    float prev_height = height;
    height = alpha * measurement + (1.0f - alpha) * prev_height;

    // Update variance
    float measurement_variance =
        (measurement - height) * (measurement - height);
    variance = alpha * measurement_variance + (1.0f - alpha) * variance;
  }

  void reset() override {
    // Stateless - nothing to reset
  }

  std::string getType() const override { return "MovingAverage"; }

private:
  size_t window_size_;
};

// Adaptive Kalman Estimator - stateless version
class AdaptiveKalmanEstimator : public IHeightEstimator {
public:
  AdaptiveKalmanEstimator(float initial_process_noise = 0.01f,
                          float initial_measurement_noise = 0.1f,
                          bool adaptive = true)
      : process_noise_(initial_process_noise),
        measurement_noise_(initial_measurement_noise), adaptive_(adaptive) {}

  void update(float &height, float &variance, float measurement) override {
    if (!std::isfinite(height)) {
      // Initialize with first measurement
      height = measurement;
      variance = measurement_noise_;
      return;
    }

    // Prediction step
    float predicted_variance = variance + process_noise_;

    // Innovation (measurement residual)
    float innovation = measurement - height;

    // Adaptive noise estimation based on current innovation
    float current_measurement_noise = measurement_noise_;
    if (adaptive_) {
      // Estimate noise from innovation magnitude
      float innovation_variance = innovation * innovation;
      current_measurement_noise =
          0.7f * measurement_noise_ + 0.3f * innovation_variance;
    }

    // Kalman gain
    float kalman_gain =
        predicted_variance / (predicted_variance + current_measurement_noise);

    // Update step
    height = height + kalman_gain * innovation;
    variance = (1 - kalman_gain) * predicted_variance;
  }

  void reset() override {
    // Stateless - nothing to reset
  }

  std::string getType() const override {
    return adaptive_ ? "AdaptiveKalman" : "Kalman";
  }

private:
  float process_noise_;
  float measurement_noise_;
  bool adaptive_;
};

// Stage that updates height map with point cloud measurements
class HeightEstimationStage : public PipelineStage<HeightEstimationStage> {
public:
  static constexpr const char *STAGE_NAME = "HeightEstimation";
  static constexpr const char *STAGE_TYPE = "Map Update";

  enum class EstimatorType {
    MEAN,
    KALMAN,
    INCREMENTAL_MEAN,
    MOVING_AVERAGE,
    ADAPTIVE_KALMAN
  };

  // Constructor
  explicit HeightEstimationStage(
      EstimatorType type = EstimatorType::INCREMENTAL_MEAN)
      : estimator_type_(type) {
    createEstimator();
  }

  void initialize() override {
    // Ensure map is initialized
    initialized_ = true;
  }

  void processImpl(ProcessingContext &ctx) {
    if (ctx.cloud().empty()) {
      // std::cout << "[HeightEstimation] Cloud is empty, skipping" <<
      // std::endl;
      return;
    }

    // std::cout << "[HeightEstimation] Processing " <<
    // ctx.cloud().points.size()
    //           << " points in ctx " << ctx.cloud().frame_id << std::endl;

    auto &map = ctx.map();

    // Only initialize if the map hasn't been initialized yet
    if (!map.isInitialized()) {
      float width = ctx.getOrDefault<float>("map_width", 20.0f);
      float height = ctx.getOrDefault<float>("map_height", 20.0f);
      float resolution = ctx.getOrDefault<float>("map_resolution", 0.1f);
      map.initialize(width, height, resolution);
      // std::cout << "[HeightEstimation] Initialized map: " << width << "x" <<
      // height
      //           << " m, resolution: " << resolution << " m" << std::endl;
    }

    size_t cells_updated = 0;
    size_t points_used = 0;
    size_t out_of_bounds = 0;
    size_t invalid_points = 0;

    // Process each point
    for (const auto &point : ctx.cloud().points) {
      if (!point.isFinite()) {
        invalid_points++;
        continue;
      }

      // Check if point is in map bounds
      if (!map.isInBounds(point.x, point.y)) {
        out_of_bounds++;
        // if (out_of_bounds == 1) {  // Log first out of bounds point
        //     std::cout << "[HeightEstimation] First out-of-bounds point: ("
        //               << point.x << ", " << point.y << ", " << point.z << ")"
        //               << std::endl;
        // }
        continue;
      }

      // Get current height and variance at this position
      float current_height = map.getHeightSafe(point.x, point.y);
      float current_variance = 1.0f;

      if (!std::isfinite(current_height)) {
        current_height = point.z;
        current_variance = 1.0f;
      } else {
        map.getVariance(point.x, point.y, current_variance);
      }

      // Update with estimator
      estimator_->update(current_height, current_variance, point.z);

      // Set updated values
      if (map.setHeight(point.x, point.y, current_height, current_variance)) {
        cells_updated++;
      }

      // Track min/max
      map.updateMinMax(point.x, point.y, point.z);

      // Increment measurement count
      map.incrementMeasurementCount(point.x, point.y);

      points_used++;
    }

    // Debug output
    // std::cout << "[HeightEstimation] Results: "
    //           << "points_in=" << ctx.cloud().points.size()
    //           << ", invalid=" << invalid_points
    //           << ", out_of_bounds=" << out_of_bounds
    //           << ", points_used=" << points_used
    //           << ", cells_updated=" << cells_updated << std::endl;

    // Update statistics
    ctx.stats().cells_updated = cells_updated;
    ctx.set("points_used_for_mapping", points_used);
    ctx.set("estimator_type", estimator_->getType());

    // Compute map statistics if requested
    if (ctx.getOrDefault<bool>("compute_map_stats", false)) {
      auto stats = map.computeStatistics();
      ctx.set("map_min_height", stats.min_height);
      ctx.set("map_max_height", stats.max_height);
      ctx.set("map_mean_height", stats.mean_height);
      ctx.set("map_valid_cells", stats.valid_cells);
    }
  }

  bool canProcess(const ProcessingContext &ctx) const override {
    return !frame.cloud().empty() && initialized_;
  }

  // Configuration
  void setEstimatorType(EstimatorType type) {
    estimator_type_ = type;
    createEstimator();
  }

  EstimatorType getEstimatorType() const { return estimator_type_; }

  void setKalmanParameters(float process_noise, float measurement_noise) {
    if (estimator_type_ == EstimatorType::KALMAN) {
      estimator_ =
          std::make_unique<KalmanEstimator>(process_noise, measurement_noise);
    }
  }

private:
  void createEstimator() {
    switch (estimator_type_) {
    case EstimatorType::MEAN:
      estimator_ = std::make_unique<MeanEstimator>();
      break;
    case EstimatorType::KALMAN:
      estimator_ = std::make_unique<KalmanEstimator>();
      break;
    case EstimatorType::INCREMENTAL_MEAN:
      estimator_ = std::make_unique<IncrementalMeanEstimator>();
      break;
    case EstimatorType::MOVING_AVERAGE:
      estimator_ = std::make_unique<MovingAverageEstimator>();
      break;
    case EstimatorType::ADAPTIVE_KALMAN:
      estimator_ = std::make_unique<AdaptiveKalmanEstimator>();
      break;
    }
  }

  std::unique_ptr<IHeightEstimator> estimator_;
  EstimatorType estimator_type_ = EstimatorType::INCREMENTAL_MEAN;
  bool initialized_ = false;
};

} // namespace height_mapping::core

#endif // HEIGHT_MAPPING_CORE_PIPELINE_STAGES_HEIGHT_ESTIMATION_STAGE_H