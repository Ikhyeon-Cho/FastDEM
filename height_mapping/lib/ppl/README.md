# ppl

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Header-only](https://img.shields.io/badge/header--only-yes-green.svg)]()

**Minimal YAML-based pipeline framework for C++17.**

```cpp
#include <ppl/ppl.h>

// 1. Define context (shared data)
struct Context {
    std::vector<float> data;
    float result;
};

// 2. Implement stages
class Normalize : public ppl::Stage<Context> {
    void configure(const YAML::Node& cfg) override {
        scale_ = cfg["scale"].as<float>(1.0f);
    }
    bool process(const std::shared_ptr<Context>& ctx) override {
        for (auto& v : ctx->data) v *= scale_;
        return true;
    }
    float scale_;
};
PPL_REGISTER_STAGE(Context, Normalize, "Normalize")

// 3. Load and run
ppl::Pipeline<Context> pipeline;
pipeline.load("pipeline.yaml");
pipeline.run(context);
```

## Features

- **Stage**: Base class with `configure()` and `process()`
- **Registry**: Auto-registration via `PPL_REGISTER_STAGE` macro
- **Pipeline**: YAML-driven stage composition
- **Profiler**: Optional timing decorator

## Installation

Header-only. Copy `include/ppl` to your project or use CMake:

```cmake
add_subdirectory(ppl)
target_link_libraries(your_target PRIVATE ppl)
```

## YAML Config

```yaml
pipeline:
  - name: Normalize
    scale: 0.5
  - name: Process
    enabled: true
```

## Requirements

- C++17
- yaml-cpp

## License

MIT License - Ikhyeon Cho
