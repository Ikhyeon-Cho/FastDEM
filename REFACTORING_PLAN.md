# Height Mapping 리팩토링 계획

## 현재 코드베이스의 주요 문제점

### 1. 코드 중복 (Code Duplication)
- `HeightMappingNode::processLidarScan()`과 `processRGBDScan()`이 거의 동일한 로직 수행
- `SensorProcessorNode::syncCallback2()`와 `syncCallback3()` 중복 코드
- Template explicit instantiation 중복 (HeightMapper.cpp:197-233)
- 센서별 처리 로직이 하드코딩되어 있음

### 2. 성능 병목 (Performance Bottlenecks)
- 매 프레임마다 전체 포인트클라우드 복사 발생
- `pcl::moveFromROSMsg()` 후 불필요한 `clear()` 호출
- Template instantiation으로 인한 컴파일 시간 증가
- 동기식 처리로 인한 처리 지연
- 중간 처리 단계마다 메모리 재할당

### 3. 메모리 관리 문제
- Object pooling 없이 계속 새로운 메모리 할당
- 중간 처리 단계마다 새로운 포인트클라우드 생성
- 명시적인 `clear()` 호출로 메모리 단편화 가능성
- Smart pointer 사용 불일치 (boost::shared_ptr vs std::shared_ptr)

### 4. 확장성 제한
- 센서 2-3개만 지원하는 하드코딩 (SensorProcessorNode)
- Point type (Laser, Color) 하드코딩
- Static frame ID 관리
- 새로운 센서 추가 시 코드 수정 필요

### 5. God Class 안티패턴
- `HeightMappingNode`가 너무 많은 책임 담당
  - TF 변환 처리
  - 포인트클라우드 필터링
  - 높이 매핑
  - 데이터 퍼블리싱
  - 타이머 관리
- 단일 책임 원칙(SRP) 위반

### 6. 의존성 문제
- 강한 결합(tight coupling) - ROS 의존성이 core 로직에 침투
- 테스트 어려움 - ROS 환경 없이 단위 테스트 불가
- 인터페이스 부재로 mock 객체 사용 불가

## 리팩토링 원칙
1. **모든 단계에서 빌드 성공 보장**
2. **기능 변경 없이 구조 개선**
3. **점진적 변경 (Incremental Changes)**
4. **테스트 가능한 구조로 전환**
5. **SOLID 원칙 적용**

## 단계별 리팩토링 계획

### Phase 1: 준비 및 기반 구축 (Build Safe)

#### Step 1.1: 테스트 환경 구축
- 현재 코드의 기능 테스트 작성
  - 포인트클라우드 처리 테스트
  - 높이 추정 알고리즘 테스트
  - TF 변환 테스트
- CI/CD 파이프라인 설정
- 빌드 검증 스크립트 작성

**구현 파일:**
```
tests/
├── unit/
│   ├── test_height_mapper.cpp
│   ├── test_point_cloud_ops.cpp
│   └── test_estimators.cpp
├── integration/
│   └── test_ros_nodes.cpp
└── scripts/
    └── build_verify.sh
```

#### Step 1.2: 인터페이스 분리
- 새로운 인터페이스 파일들 추가 (기존 코드 유지)
- Core 로직과 ROS 의존성 분리

**새로운 인터페이스:**
```cpp
// interfaces/point_cloud_processor_interface.h
template<typename PointT>
class IPointCloudProcessor {
public:
    virtual ~IPointCloudProcessor() = default;
    virtual CloudPtr process(const CloudPtr& input) = 0;
    virtual void configure(const Config& config) = 0;
};

// interfaces/sensor_data_handler_interface.h
class ISensorDataHandler {
public:
    virtual ~ISensorDataHandler() = default;
    virtual void handleSensorData(const SensorData& data) = 0;
    virtual bool isReady() const = 0;
};
```

### Phase 2: 코드 중복 제거 (Incremental Refactoring)

#### Step 2.1: Template 기반 통합
- 중복된 처리 로직을 template 클래스로 통합
- 기존 코드 유지하며 새로운 processor 추가

**구현:**
```cpp
// processors/unified_point_cloud_processor.hpp
template<typename PointT>
class UnifiedPointCloudProcessor : public IPointCloudProcessor<PointT> {
private:
    CloudPtr transformToBase(const CloudPtr& cloud, const Transform& tf);
    CloudPtr filterHeight(const CloudPtr& cloud);
    CloudPtr filterRange(const CloudPtr& cloud);

public:
    CloudPtr process(const CloudPtr& input) override {
        // 통합된 처리 로직
        auto transformed = transformToBase(input, current_transform_);
        auto filtered = filterHeight(transformed);
        return filterRange(filtered);
    }
};
```

#### Step 2.2: 센서 처리 통합
- N개 센서를 지원하는 동적 시스템 구현
- Factory 패턴으로 센서 생성

**구현:**
```cpp
// sensors/sensor_fusion.h
class SensorFusion {
private:
    std::vector<std::unique_ptr<ISensorDataHandler>> handlers_;

public:
    void addSensor(const std::string& type, const Config& config);
    void processSensorData(size_t sensor_id, const PointCloud& data);
    PointCloud getFusedData() const;
};
```

### Phase 3: 성능 최적화 (Memory & CPU)

#### Step 3.1: Object Pooling 구현
- 포인트클라우드 메모리 재사용
- Lock-free pool 구현

**구현:**
```cpp
// memory/point_cloud_pool.h
template<typename PointT>
class PointCloudPool {
private:
    std::queue<CloudPtr> available_;
    std::mutex mutex_;
    size_t max_size_;

public:
    CloudPtr acquire() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (available_.empty()) {
            return boost::make_shared<pcl::PointCloud<PointT>>();
        }
        auto cloud = available_.front();
        available_.pop();
        cloud->clear();
        return cloud;
    }

    void release(CloudPtr cloud) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (available_.size() < max_size_) {
            available_.push(cloud);
        }
    }
};
```

#### Step 3.2: Move Semantics 적용
- 불필요한 복사 제거
- RValue reference 활용

**변경 예시:**
```cpp
// Before
auto cloud_processed = processCloud(cloud_input);

// After
auto cloud_processed = processCloud(std::move(cloud_input));
```

### Phase 4: 아키텍처 개선 (SOLID Principles)

#### Step 4.1: 책임 분리
- God Class를 작은 클래스들로 분할
- 각 클래스는 단일 책임만 담당

**새로운 클래스 구조:**
```cpp
// managers/tf_manager.h
class TFManager {
public:
    Transform lookupTransform(const std::string& from, const std::string& to);
    void updateTransform(const std::string& frame, const Transform& tf);
};

// filters/cloud_filter_chain.h
class CloudFilterChain {
private:
    std::vector<std::unique_ptr<ICloudFilter>> filters_;
public:
    void addFilter(std::unique_ptr<ICloudFilter> filter);
    CloudPtr apply(const CloudPtr& input);
};

// publishers/map_publisher.h
class MapPublisher {
public:
    void publishHeightMap(const HeightMap& map);
    void publishPointCloud(const PointCloud& cloud);
};
```

#### Step 4.2: 의존성 주입
- Constructor injection 패턴 적용
- 테스트 가능한 구조

**구현:**
```cpp
class HeightMappingCore {
private:
    std::unique_ptr<IPointCloudProcessor> processor_;
    std::unique_ptr<IHeightEstimator> estimator_;
    std::unique_ptr<IMapPublisher> publisher_;

public:
    HeightMappingCore(
        std::unique_ptr<IPointCloudProcessor> processor,
        std::unique_ptr<IHeightEstimator> estimator,
        std::unique_ptr<IMapPublisher> publisher)
        : processor_(std::move(processor)),
          estimator_(std::move(estimator)),
          publisher_(std::move(publisher)) {}
};
```

### Phase 5: 확장성 개선

#### Step 5.1: Plugin 시스템
- 동적 센서 등록
- Runtime configuration

**구현:**
```cpp
// plugins/sensor_plugin_interface.h
class ISensorPlugin {
public:
    virtual std::string getType() const = 0;
    virtual void initialize(const YAML::Node& config) = 0;
    virtual CloudPtr process(const RawData& data) = 0;
};

// plugins/sensor_registry.h
class SensorRegistry {
private:
    std::map<std::string, std::function<std::unique_ptr<ISensorPlugin>()>> factories_;

public:
    template<typename T>
    void registerSensor(const std::string& type) {
        factories_[type] = []() { return std::make_unique<T>(); };
    }

    std::unique_ptr<ISensorPlugin> create(const std::string& type);
};
```

#### Step 5.2: Async Processing
- Pipeline 패턴 구현
- Thread pool 활용

**구현:**
```cpp
// processing/async_pipeline.h
class AsyncPipeline {
private:
    ThreadPool thread_pool_;
    std::vector<Stage> stages_;

public:
    template<typename Func>
    void addStage(Func&& func) {
        stages_.emplace_back(std::forward<Func>(func));
    }

    std::future<CloudPtr> process(CloudPtr input) {
        return thread_pool_.enqueue([this, input]() {
            CloudPtr result = input;
            for (const auto& stage : stages_) {
                result = stage(result);
            }
            return result;
        });
    }
};
```

## 빌드 검증 스크립트

```bash
#!/bin/bash
# scripts/build_verify.sh

set -e  # Exit on error

echo "========================================="
echo "Height Mapping Build Verification"
echo "========================================="

# Clean build
echo "Step 1: Clean previous build..."
catkin clean -y

# Build core library
echo "Step 2: Building height_mapping_core..."
catkin build height_mapping_core --no-deps

# Build ROS nodes
echo "Step 3: Building height_mapping_ros..."
catkin build height_mapping_ros

# Run unit tests
echo "Step 4: Running unit tests..."
catkin test height_mapping_core
catkin test height_mapping_ros

# Check for warnings
echo "Step 5: Checking for compiler warnings..."
catkin build --cmake-args -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic"

echo "========================================="
echo "Build Verification Complete!"
echo "========================================="
```

## 구현 우선순위

1. **High Priority (즉시 구현)**
   - Phase 1.1: 테스트 환경 구축
   - Phase 2.1: Template 기반 통합
   - Phase 3.1: Object Pooling

2. **Medium Priority (1-2주 내)**
   - Phase 1.2: 인터페이스 분리
   - Phase 2.2: 센서 처리 통합
   - Phase 4.1: 책임 분리

3. **Low Priority (장기 계획)**
   - Phase 4.2: 의존성 주입
   - Phase 5.1: Plugin 시스템
   - Phase 5.2: Async Processing

## 성공 지표

### 성능 개선
- 메모리 사용량 30% 감소
- 처리 속도 20% 향상
- 컴파일 시간 15% 단축

### 코드 품질
- 코드 중복 50% 감소
- 테스트 커버리지 80% 이상
- 순환 복잡도(Cyclomatic Complexity) 10 이하

### 유지보수성
- 새 센서 추가 시간: 1시간 이내
- 단위 테스트 실행 시간: 30초 이내
- 빌드 시간: 2분 이내

## 리스크 및 대응 방안

### 리스크 1: 기존 기능 손상
**대응:**
- 모든 변경 전 테스트 작성
- Feature flag로 점진적 롤아웃
- 이전 버전 태그 유지

### 리스크 2: 성능 저하
**대응:**
- 벤치마크 테스트 구축
- 프로파일링 도구 활용
- A/B 테스트 수행

### 리스크 3: 호환성 문제
**대응:**
- Semantic Versioning 적용
- Deprecation warning 제공
- Migration guide 작성

## 마일스톤

### M1: Foundation (Week 1-2)
- [ ] 테스트 환경 구축 완료
- [ ] 빌드 검증 스크립트 작성
- [ ] 기본 인터페이스 정의

### M2: Core Refactoring (Week 3-4)
- [ ] Template 기반 통합 완료
- [ ] 코드 중복 50% 감소
- [ ] 단위 테스트 커버리지 60%

### M3: Performance (Week 5-6)
- [ ] Object pooling 구현
- [ ] 메모리 사용량 20% 감소
- [ ] 처리 속도 15% 향상

### M4: Architecture (Week 7-8)
- [ ] 책임 분리 완료
- [ ] 의존성 주입 적용
- [ ] 테스트 커버리지 80%

### M5: Advanced Features (Week 9-10)
- [ ] Plugin 시스템 구축
- [ ] Async processing 구현
- [ ] 최종 성능 목표 달성