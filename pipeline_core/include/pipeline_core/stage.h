/*
 * stage.h
 *
 * PUBLIC API - Base class for pipeline stages.
 *
 * Users must inherit from Stage to create custom processing stages.
 * Each stage represents a single processing step in the pipeline.
 *
 * Example implementation:
 *   class MyStage : public pipeline::Stage {
 *   public:
 *     MyStage() : Stage("MyStage") {}
 *
 *     void configure(const std::map<std::string, std::string>& params) override {
 *       // Parse parameters
 *       if (params.find("threshold") != params.end()) {
 *         threshold_ = std::stod(params.at("threshold"));
 *       }
 *     }
 *
 *   protected:
 *     void processImpl(pipeline::Context& ctx) override {
 *       // Implement your processing logic here
 *       // Access and modify data through ctx
 *     }
 *
 *   private:
 *     double threshold_ = 1.0;
 *   };
 *
 *   // Register the stage for factory creation
 *   REGISTER_STAGE(MyStage);
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef PIPELINE_CORE_STAGE_H
#define PIPELINE_CORE_STAGE_H

#include "pipeline_core/context.h"
#include <map>
#include <memory>
#include <string>

namespace pipeline {

class Stage {
public:
  using Ptr = std::unique_ptr<Stage>;

  Stage(const std::string &name) : name_(name) { initialize(); }

  virtual ~Stage() = default;

  void process(Context &ctx) {
    if (!isEnabled() || !canProcess(ctx))
      return;
    processImpl(ctx);
  }

  // Configuration interface for parameter tuning
  // Default: no configuration needed
  // Override in derived classes that need parameters
  virtual void configure(const std::map<std::string, std::string> &params) {}

  // Accessors
  std::string getName() const { return name_; }
  bool isEnabled() const { return enabled_; }
  void setEnabled(bool enabled) { enabled_ = enabled; }

  virtual bool canProcess(const Context &ctx) const { return true; }
  virtual void initialize() {}

protected:
  virtual void processImpl(Context &ctx) = 0; // Override in specific stages

private:
  std::string name_;
  bool enabled_ = true;
};

} // namespace pipeline

#endif // PIPELINE_CORE_STAGE_H