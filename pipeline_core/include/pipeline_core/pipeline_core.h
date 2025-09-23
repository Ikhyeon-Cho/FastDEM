/*
 * pipeline_core.h
 *
 * Main public API header for pipeline_core library.
 * Include this single header to use the complete pipeline framework.
 *
 *  Created on: Dec 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef PIPELINE_CORE_H
#define PIPELINE_CORE_H

// ============================================================================
// PUBLIC API - Core components for users
// ============================================================================

// Base classes for extension
#include "pipeline_core/stage.h"          // Inherit to create custom stages
#include "pipeline_core/context.h"        // Inherit to create custom contexts

// Configuration and construction
#include "pipeline_core/config.h"         // Load/parse pipeline configurations
#include "pipeline_core/pipeline_builder.h"// Build pipelines from configs

// Pipeline execution
#include "pipeline_core/pipeline.h"       // Execute pipelines

// Stage registration
#include "pipeline_core/stage_registry.h" // REGISTER_STAGE macro only

// Note: exceptions.h is not included by default (internal use)

// ============================================================================
// QUICK START GUIDE
// ============================================================================

/*
 * 1. IMPLEMENT A CUSTOM STAGE:
 *
 *    #include <pipeline_core/pipeline_core.h>
 *
 *    class MyStage : public pipeline::Stage {
 *    public:
 *      MyStage() : Stage("MyStage") {}
 *
 *      void configure(const std::map<std::string, std::string>& params) override {
 *        // Parse configuration parameters
 *      }
 *
 *    protected:
 *      void processImpl(pipeline::Context& ctx) override {
 *        // Your processing logic here
 *      }
 *    };
 *
 *    REGISTER_STAGE(MyStage);
 *
 * 2. CREATE A CUSTOM CONTEXT:
 *
 *    class MyContext : public pipeline::Context {
 *    public:
 *      std::shared_ptr<MyData> data;
 *    };
 *
 * 3. BUILD AND RUN PIPELINE:
 *
 *    // From configuration file (recommended)
 *    auto pipeline = pipeline::PipelineBuilder::fromFile("pipeline.yaml");
 *
 *    // Execute pipeline
 *    MyContext ctx;
 *    ctx.data = std::make_shared<MyData>();
 *    pipeline->process(ctx);
 *
 * 4. YAML CONFIGURATION FORMAT:
 *
 *    pipeline:
 *      stop_on_error: true
 *      stages:
 *        - name: "MyStage"
 *          enabled: true
 *          params:
 *            threshold: 1.0
 *            mode: "fast"
 */

#endif // PIPELINE_CORE_H