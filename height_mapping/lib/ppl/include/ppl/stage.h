/*
 * stage.h - Stage interface for ppl (Pipeline Library)
 *
 * Template-based stage interface that accepts user-defined context types.
 * Each stage receives YAML::Node directly for type-safe configuration.
 *
 * Usage:
 *   class MyStage : public ppl::Stage<MyContext> {
 *       void configure(const YAML::Node& config) override { ... }
 *       bool process(const std::shared_ptr<MyContext>& ctx) override { ... }
 *   };
 *
 * Created on: Dec 2024
 * Author: Ikhyeon Cho
 * Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 * Email: tre0430@korea.ac.kr
 */

#ifndef PPL_STAGE_H
#define PPL_STAGE_H

#include <memory>
#include <string>
#include <yaml-cpp/yaml.h>

namespace ppl {

/**
 * @brief Base class for all pipeline stages
 *
 * @tparam TContext User-defined context type (data bus)
 */
template <typename TContext>
class Stage {
public:
  using Ptr = std::shared_ptr<Stage<TContext>>;

  Stage() = default;
  virtual ~Stage() = default;

  // Non-copyable, non-movable
  Stage(const Stage&) = delete;
  Stage& operator=(const Stage&) = delete;
  Stage(Stage&&) = delete;
  Stage& operator=(Stage&&) = delete;

  /**
   * @brief Configure stage with YAML parameters
   *
   * Called once during pipeline initialization.
   * Override to parse stage-specific parameters into typed config struct.
   *
   * @param config YAML node containing stage parameters
   */
  virtual void configure(const YAML::Node& /*config*/) {}

  /**
   * @brief Process the context
   *
   * Called for each frame/iteration. Must be implemented by derived classes.
   *
   * @param context Shared pointer to the context (data bus)
   * @return true if processing succeeded, false to stop pipeline
   */
  virtual bool process(const std::shared_ptr<TContext>& context) = 0;

  /**
   * @brief Get stage name (instance name from YAML)
   */
  const std::string& name() const { return name_; }

  /**
   * @brief Set stage name (called by Pipeline during load)
   */
  void setName(const std::string& name) { name_ = name; }

private:
  std::string name_;
};

}  // namespace ppl

#endif  // PPL_STAGE_H
