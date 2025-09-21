/*
 * stage.h
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

  Stage(const std::string &name, const std::string &type)
      : name_(name), type_(type) {
    initialize();
  }

  virtual ~Stage() = default;

  void process(Context &ctx) {
    if (!isEnabled() || !canProcess(ctx))
      return;
    processImpl(ctx);
  }

  // Configuration interface for parameter tuning
  virtual void configure(const std::map<std::string, std::string> &params) {
    // Default: no configuration needed
    // Override in derived classes that need parameters
  }

  // Accessors
  std::string getName() const { return name_; }
  std::string getType() const { return type_; }
  bool isEnabled() const { return enabled_; }
  void setEnabled(bool enabled) { enabled_ = enabled; }

  virtual bool canProcess(const Context &ctx) const { return true; }
  virtual void initialize() {}

protected:
  virtual void processImpl(Context &ctx) = 0; // Override in specific stages

private:
  std::string name_;
  std::string type_;
  bool enabled_ = true;
};

} // namespace pipeline

#endif // PIPELINE_CORE_STAGE_H