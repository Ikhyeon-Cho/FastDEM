/*
 * stage.h
 *
 * PUBLIC API - Base class for pipeline stages.
 *
 * Users must inherit from Stage to create custom processing stages.
 * Each stage represents a single processing step in the pipeline.
 *
 * ###################################################################################
 * Example implementation:
 *
 * Header file (my_stage.h):
 *   class MyStage : public flowpipe::Stage {
 *   public:
 *     MyStage() : Stage("MyStage") {}
 *
 *     void configure(const std::map<std::string, std::string>& params) override
 *{
 *       // Parse parameters
 *       if (params.find("threshold") != params.end()) {
 *         threshold_ = std::stod(params.at("threshold"));
 *       }
 *     }
 *
 *   protected:
 *     void processImpl(flowpipe::Context& ctx) override {
 *       // Implement your processing logic here
 *       // Access and modify data through ctx
 *     }
 *
 *   private:
 *     double threshold_ = 1.0;
 *   };
 *
 * Implementation file (my_stage.cpp):
 *   #include "my_stage.h"
 *   #include "flowpipe/stage_registration.h"  // Only needed in .cpp
 *
 *   // Register the stage for factory creation
 *   REGISTER_STAGE(MyStage);
 * ###################################################################################
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef FLOWPIPE_STAGE_H
#define FLOWPIPE_STAGE_H

#include <map>
#include <memory>
#include <string>

#include "flowpipe/context.h"

namespace flowpipe {

class Stage {
 public:
  using Ptr = std::unique_ptr<Stage>;

  Stage(const std::string &name) : name_(name) { initialize(); }

  virtual ~Stage() = default;

  void process(Context &ctx) {
    if (!isEnabled() || !canProcess(ctx)) return;
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
  virtual void processImpl(Context &ctx) = 0;  // Override in specific stages

  // Parameter parsing utilities
  static bool loadParam(const std::map<std::string, std::string> &params,
                        const std::string &key, float &value) {
    auto it = params.find(key);
    if (it != params.end()) {
      value = std::stof(it->second);
      return true;
    }
    return false;
  }

  static bool loadParam(const std::map<std::string, std::string> &params,
                        const std::string &key, double &value) {
    auto it = params.find(key);
    if (it != params.end()) {
      value = std::stod(it->second);
      return true;
    }
    return false;
  }

  static bool loadParam(const std::map<std::string, std::string> &params,
                        const std::string &key, int &value) {
    auto it = params.find(key);
    if (it != params.end()) {
      value = std::stoi(it->second);
      return true;
    }
    return false;
  }

  static bool loadParam(const std::map<std::string, std::string> &params,
                        const std::string &key, size_t &value) {
    auto it = params.find(key);
    if (it != params.end()) {
      value = static_cast<size_t>(std::stoi(it->second));
      return true;
    }
    return false;
  }

  static bool loadParam(const std::map<std::string, std::string> &params,
                        const std::string &key, bool &value) {
    auto it = params.find(key);
    if (it != params.end()) {
      value = (it->second == "true" || it->second == "1");
      return true;
    }
    return false;
  }

  static bool loadParam(const std::map<std::string, std::string> &params,
                        const std::string &key, std::string &value) {
    auto it = params.find(key);
    if (it != params.end()) {
      value = it->second;
      return true;
    }
    return false;
  }

 private:
  std::string name_;
  bool enabled_ = true;
};

}  // namespace flowpipe

#endif  // FLOWPIPE_STAGE_H