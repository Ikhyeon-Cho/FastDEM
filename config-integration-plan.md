# Configuration System Integration Plan

## Current Issues
1. **Disconnected Configuration**: YAML files exist but aren't connected to the code
2. **Hardcoded Pipeline**: MappingEngine uses hardcoded stages instead of YAML
3. **Multiple Config Files**: Confusing separation between ros_params, engine_config, and pipeline_config
4. **ROS Node Complexity**: ROS node handles configuration logic instead of just ROS interfaces

## Proposed Architecture

### Design Principles
- **Single Source of Truth**: One unified configuration file
- **Separation of Concerns**: ROS node handles only ROS interfaces
- **Self-Contained Engine**: MappingEngine manages its own configuration
- **Fallback Strategy**: Default pipeline if configuration fails

### Configuration Structure
```yaml
# unified_config.yaml
engine:
  map:
    width: 10.0
    height: 10.0
    resolution: 0.1
    frame_id: "map"
  runtime:
    thread_safe: true
    auto_reset_on_error: false

pipeline:
  stop_on_error: true
  stages:
    - name: "MapOriginUpdate"
      enabled: true
      params:
        update_threshold: 2.0
    - name: "PointCloudTransform"
      enabled: true
      params:
        target_frame: "base_link"
    - name: "VoxelFilter"
      enabled: true
      params:
        voxel_size: 0.05
    - name: "PassthroughFilter"
      enabled: true
      params:
        min_distance: 0.3
        max_distance: 20.0
    - name: "Raycasting"
      enabled: true
      params:
        ray_length: 5.0
    - name: "HeightEstimation"
      enabled: true
      params:
        estimator_type: "kalman"
        process_noise: 0.01
        measurement_noise: 0.05
```

## Implementation Steps

### Phase 1: MappingEngine Refactoring
**Goal**: Enable MappingEngine to load configuration from YAML

#### 1.1 Modify MappingEngine Constructor
```cpp
// mapping_engine.h
class MappingEngine {
public:
  // New constructor accepting config file path
  MappingEngine(const std::string& config_file,
                std::shared_ptr<ITransformProvider> transform_provider);

  // Keep old constructor for backward compatibility (uses defaults)
  MappingEngine(const EngineConfig& config,
                std::shared_ptr<ITransformProvider> transform_provider);

private:
  void loadConfiguration(const std::string& config_file);
  void setupPipeline();
  void setupDefaultPipeline();

  YAML::Node pipeline_config_node_;
  bool use_default_pipeline_{false};
};
```

#### 1.2 Implement Configuration Loading
```cpp
// mapping_engine.cpp
void MappingEngine::loadConfiguration(const std::string& config_file) {
  if (config_file.empty()) {
    config_ = EngineConfig{};
    use_default_pipeline_ = true;
    return;
  }

  try {
    auto yaml = YAML::LoadFile(config_file);

    // Parse engine configuration
    if (yaml["engine"]) {
      auto engine = yaml["engine"];
      if (engine["map"]) {
        config_.map.width = engine["map"]["width"].as<double>();
        config_.map.height = engine["map"]["height"].as<double>();
        config_.map.resolution = engine["map"]["resolution"].as<double>();
        config_.map.frame_id = engine["map"]["frame_id"].as<std::string>();
      }
      if (engine["runtime"]) {
        config_.engine.thread_safe = engine["runtime"]["thread_safe"].as<bool>();
        config_.engine.auto_reset_on_error = engine["runtime"]["auto_reset_on_error"].as<bool>();
      }
    }

    // Store pipeline configuration for later use
    if (yaml["pipeline"]) {
      pipeline_config_node_ = yaml["pipeline"];
      use_default_pipeline_ = false;
    } else {
      use_default_pipeline_ = true;
    }

  } catch (const std::exception& e) {
    std::cerr << "Failed to load config: " << e.what() << std::endl;
    config_ = EngineConfig{};
    use_default_pipeline_ = true;
  }
}
```

#### 1.3 Create Stage Factory
```cpp
// mapping_engine.cpp
std::unique_ptr<pipeline::Stage<MappingContext>>
MappingEngine::createStage(const std::string& name) {
  // Stage factory implementation
  if (name == "MapOriginUpdate") {
    return std::make_unique<MapOriginUpdateStage>();
  } else if (name == "PointCloudTransform") {
    return std::make_unique<PointCloudTransformStage>();
  } else if (name == "VoxelFilter") {
    return std::make_unique<VoxelFilterStage>();
  } else if (name == "PassthroughFilter") {
    return std::make_unique<PassthroughFilterStage>();
  } else if (name == "Raycasting") {
    return std::make_unique<RaycastingStage>();
  } else if (name == "HeightEstimation") {
    return std::make_unique<HeightEstimationStage>();
  } else if (name == "MultiSensorSync") {
    return std::make_unique<MultiSensorSyncStage>();
  } else if (name == "GlobalMapping") {
    return std::make_unique<GlobalMappingStage>();
  }

  throw std::runtime_error("Unknown stage: " + name);
}
```

#### 1.4 Modify Pipeline Setup
```cpp
// mapping_engine.cpp
void MappingEngine::setupPipeline() {
  if (use_default_pipeline_) {
    setupDefaultPipeline();
    return;
  }

  try {
    // Create pipeline from YAML configuration
    pipeline::PipelineBuilder builder;

    if (pipeline_config_node_["stop_on_error"]) {
      builder.stopOnError(pipeline_config_node_["stop_on_error"].as<bool>());
    }

    if (pipeline_config_node_["stages"]) {
      for (const auto& stage_node : pipeline_config_node_["stages"]) {
        std::string name = stage_node["name"].as<std::string>();
        bool enabled = stage_node["enabled"].as<bool>(true);

        if (!enabled) continue;

        auto stage = createStage(name);
        if (stage_node["params"]) {
          // Pass params to stage
          pipeline::StageConfig config;
          config.name = name;
          config.enabled = enabled;
          config.params = stage_node["params"];
          stage->configure(config);
        }

        builder.addStage(std::move(stage));
      }
    }

    mapping_pipeline_ = builder.build();
    std::cout << "Pipeline loaded from configuration" << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "Failed to setup pipeline from config: " << e.what() << std::endl;
    setupDefaultPipeline();
  }
}
```

### Phase 2: ROS Node Simplification
**Goal**: ROS node only handles ROS interfaces

#### 2.1 Simplify ROS Node
```cpp
// height_mapping_node.cpp
class MappingNode {
private:
  void initialize() {
    // Get configuration file path from ROS parameter
    std::string config_file;
    private_nh_.param("config_file", config_file, std::string(""));

    // If not specified, try to find default config
    if (config_file.empty()) {
      config_file = ros::package::getPath("height_mapping_ros") +
                    "/config/unified_config.yaml";
      if (!std::filesystem::exists(config_file)) {
        config_file = "";  // Will use hardcoded defaults
      }
    }

    // Create transform provider (ROS responsibility)
    auto tf_provider = std::make_shared<TF2TransformProvider>(tf_buffer_);

    // Create mapping engine with config file
    try {
      mapper_ = std::make_unique<core::MappingEngine>(config_file, tf_provider);
      ROS_INFO("MappingEngine initialized with config: %s",
               config_file.empty() ? "default" : config_file.c_str());
    } catch (const std::exception& e) {
      ROS_ERROR("Failed to initialize MappingEngine: %s", e.what());
      throw;
    }

    // Setup ROS publishers and subscribers
    setupROSInterfaces();
  }

  void setupROSInterfaces() {
    // Get ROS-specific parameters
    std::string input_topic, output_topic;
    double publish_rate;

    private_nh_.param("input_topic", input_topic,
                      std::string("/velodyne/tilted/velodyne_points"));
    private_nh_.param("output_topic", output_topic,
                      std::string("/height_map"));
    private_nh_.param("publish_rate", publish_rate, 10.0);

    // Create subscriber and publisher
    cloud_sub_ = nh_.subscribe(input_topic, 1,
                                &MappingNode::cloudCallback, this);
    map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(output_topic, 1);

    // Setup timer for periodic map publishing
    map_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate),
                                  &MappingNode::mapTimerCallback, this);
  }
};
```

#### 2.2 Update Launch File
```xml
<!-- height_mapping.launch -->
<launch>
  <!-- Configuration file argument -->
  <arg name="config" default="$(find height_mapping_ros)/config/unified_config.yaml"/>

  <!-- ROS-specific arguments -->
  <arg name="input_topic" default="/velodyne/tilted/velodyne_points"/>
  <arg name="output_topic" default="/height_map"/>
  <arg name="publish_rate" default="10.0"/>

  <node name="height_mapping" pkg="height_mapping_ros" type="height_mapping_node" output="screen">
    <!-- Core configuration -->
    <param name="config_file" value="$(arg config)"/>

    <!-- ROS interface parameters -->
    <param name="input_topic" value="$(arg input_topic)"/>
    <param name="output_topic" value="$(arg output_topic)"/>
    <param name="publish_rate" value="$(arg publish_rate)"/>
  </node>
</launch>
```

### Phase 3: Configuration Migration
**Goal**: Create unified configuration files from existing separate configs

#### 3.1 Create Default Unified Config
Merge existing ros_params.yaml, engine_config.yaml, and pipeline_config.yaml into unified_config.yaml

#### 3.2 Create Configuration Presets
- `config/presets/fast.yaml` - Optimized for speed
- `config/presets/accurate.yaml` - Optimized for accuracy
- `config/presets/minimal.yaml` - Minimal pipeline for testing

### Phase 4: Testing & Validation
**Goal**: Ensure both YAML and fallback configurations work

#### 4.1 Test Cases
1. Load with valid unified config
2. Load with missing config file (use defaults)
3. Load with invalid YAML (fallback to defaults)
4. Load with partial config (merge with defaults)
5. Test each pipeline stage configuration

#### 4.2 Validation Tools
Create configuration validator script:
```bash
rosrun height_mapping validate_config config/unified_config.yaml
```

## Implementation Order

1. **Week 1**: MappingEngine refactoring (Phase 1)
   - Add new constructor
   - Implement configuration loading
   - Create stage factory
   - Update pipeline setup

2. **Week 2**: ROS Node simplification (Phase 2)
   - Simplify node initialization
   - Update launch files
   - Test integration

3. **Week 3**: Configuration migration (Phase 3)
   - Create unified configs
   - Create presets
   - Update documentation

4. **Week 4**: Testing & validation (Phase 4)
   - Implement test cases
   - Create validation tools
   - Update CI/CD

## Benefits

1. **Simplicity**: Single configuration file for entire system
2. **Flexibility**: Easy to change pipeline without recompiling
3. **Maintainability**: Clear separation of concerns
4. **Testability**: Engine can be tested without ROS
5. **Extensibility**: Easy to add new stages or configurations

## Backward Compatibility

- Keep old constructor for programmatic use
- Provide migration script for old config files
- Default behavior unchanged if no config specified

## Success Criteria

- [ ] MappingEngine loads pipeline from YAML
- [ ] ROS node only handles ROS interfaces
- [ ] Single unified configuration file works
- [ ] Fallback to defaults when config missing
- [ ] All existing functionality preserved
- [ ] Configuration validation tool available
- [ ] Documentation updated