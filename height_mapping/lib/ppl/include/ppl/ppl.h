/*
 * ppl.h - Main header for ppl (Pipeline Library)
 *
 * This is the single header you need to include to use ppl.
 * It includes all necessary components:
 * - Stage: Base class for pipeline stages
 * - Registry: Auto-registration factory
 * - Pipeline: YAML-based pipeline loader and executor
 * - Profiler: Performance measurement decorator
 *
 * Quick Start:
 *   #include <ppl/ppl.h>
 *
 *   // 1. Define your context (data bus)
 *   struct MyContext {
 *       PointCloud::Ptr cloud;
 *       Pose pose;
 *   };
 *
 *   // 2. Implement a stage
 *   class MyStage : public ppl::Stage<MyContext> {
 *       void configure(const YAML::Node& config) override { ... }
 *       bool process(const std::shared_ptr<MyContext>& ctx) override { ... }
 *   };
 *   PPL_REGISTER_STAGE(MyContext, MyStage, "MyStage")
 *
 *   // 3. Load and run pipeline
 *   ppl::Pipeline<MyContext> pipeline;
 *   pipeline.load("config.yaml");
 *
 *   auto ctx = std::make_shared<MyContext>();
 *   pipeline.run(ctx);
 *
 * Created on: Dec 2024
 * Author: Ikhyeon Cho
 * Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 * Email: tre0430@korea.ac.kr
 */

#ifndef PPL_H
#define PPL_H

#include "stage.h"
#include "registry.h"
#include "pipeline.h"
#include "profiler.h"

#endif  // PPL_H
