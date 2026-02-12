<div align="center">

# FastDEM

<a href="https://github.com/Ikhyeon-Cho/FastDEM/tree/main"><img src="https://img.shields.io/badge/C++17-00599C?logo=cplusplus&logoColor=white" /></a>
<a href="https://github.com/Ikhyeon-Cho/FastDEM/tree/ros1"><img src="https://img.shields.io/badge/ROS1-Noetic-blue" /></a>
<a href="https://github.com/Ikhyeon-Cho/FastDEM/tree/ros2"><img src="https://img.shields.io/badge/ROS2-Humble-blue" /></a>
<a href=""><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
<br>
<a href="https://github.com/Ikhyeon-Cho/FastDEM/actions/workflows/build.yml"><img src="https://github.com/Ikhyeon-Cho/FastDEM/actions/workflows/build.yml/badge.svg" /></a>
<!-- <br>
<a href=""><img src="https://img.shields.io/badge/YouTube-Demo-FF0000?logo=youtube&logoColor=white" /></a>
<a href=""><img src="https://img.shields.io/badge/FastDEM-github.io-8B0029" /></a> -->

> ***Real-time elevation mapping** on **embedded** CPUs — **100+ Hz** on Jetson Orin*

**[Quick Start](#quick-start)** · **[ROS1](https://github.com/Ikhyeon-Cho/FastDEM/tree/ros1)** · **[ROS2](https://github.com/Ikhyeon-Cho/FastDEM/tree/ros2)**

<br>

<p align="center">
  <img src="assets/fastdem_laser_local.gif" width="49%" />
  <img src="assets/fastdem_rgbd_local.gif" width="48.3%" />
  <img src="assets/fastdem_rgbd_global.gif" width="97.5%" />
</p>

</div>

---

## Features

* **Fast** — Real-time on embedded CPUs. Single thread, without GPU.
* **Lightweight** — Minimal dependencies. No OpenCV, PCL, or Open3D.
* **Library-First** — Clean C++ API, not a ROS node. Online (with TF) or offline (explicit transforms) support.
* **Sensor-Aware** — Physics-based uncertainty models for LiDAR and RGB-D.
* **Multiple Estimators** — Kalman Filter, Welford Mean, P² Quantile.
* **Local + Global Mapping** — Robot-centric or map-centric modes.
* **Post-processing functions** — Raycasting, Inpainting, Spike removal, Uncertainty fusion, etc.

---

## How Fast?

<p align="center">The mapping itself runs at <b>~130 Hz</b> — fast enough to leave ample headroom for post-processing.</p>

<p align="center">
  <img src="assets/fastdem_jetson_benchmark.svg" width="95%" />
</p>

<p align="center"><i>Measured with Velodyne VLP-16 (~30K pts/scan) · 15×15 m map at 0.1 m resolution</i></p>

---

## Quick Start

### 1. Installation

**Install dependencies**

```bash
sudo apt install libeigen3-dev libyaml-cpp-dev libspdlog-dev
```

**Clone and build**

```bash
git clone https://github.com/Ikhyeon-Cho/FastDEM.git
cd FastDEM/fastdem
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### 2. Standalone C++ Usage

Ideal for custom SLAM pipelines or non-ROS systems.

```cpp
#include <fastdem/fastdem.hpp>

int main() {
    // 1. Setup map and mapper
    fastdem::ElevationMap map;
    map.setGeometry(15.0f, 15.0f, 0.1f);

    fastdem::FastDEM mapper(map);
    mapper.setHeightFilter(-1.0f, 2.0f)
          .setDistanceFilter(0.5f, 10.0f)
          .setEstimatorType(fastdem::EstimationType::Kalman)
          .setSensorModel(fastdem::SensorType::LiDAR);
    // -- or: FastDEM mapper(map, MappingConfig::load("config/default.yaml"));

    // 2. Integration Loop
    while (true) {
        // T_base_sensor: Sensor extrinsic (Sensor -> Robot)
        // T_world_base: Robot pose (Robot -> World)
        mapper.integrate(cloud, T_base_sensor, T_world_base);

        // Access the 2.5D elevation map
        float elevation = map.elevationAt(position);
    }
}
```

See [`fastdem/examples/`](fastdem/examples/) for more usage patterns.

### 3. Configuration

All default settings are in [`config/default.yaml`](fastdem/config/default.yaml).

### 4. Run with ROS

See [ros1](https://github.com/Ikhyeon-Cho/FastDEM/tree/ros1) and [ros2](https://github.com/Ikhyeon-Cho/FastDEM/tree/ros2) branches. These are just a thin ROS wrapper of core `fastdem::FastDEM` features.

---

## Citation

FastDEM was originally developed for the following research:

**['Learning Self-supervised Traversability with Navigation Experiences of Mobile Robots'](https://github.com/Ikhyeon-Cho/LeSTA)**
*IEEE Robotics and Automation Letters (RA-L), 2024*

```bibtex
@article{cho2024learning,
  title={Learning Self-Supervised Traversability With Navigation Experiences of Mobile Robots: A Risk-Aware Self-Training Approach},
  author={Cho, Ikhyeon and Chung, Woojin},
  journal={IEEE Robotics and Automation Letters},
  year={2024},
  volume={9},
  number={5},
  pages={4122-4129},
  doi={10.1109/LRA.2024.3376148}
}
```

---

<div align="center">

**[Report Bug](https://github.com/Ikhyeon-Cho/FastDEM/issues)** · **[Pull Request](https://github.com/Ikhyeon-Cho/FastDEM/pulls)**

BSD-3-Clause License © [Ikhyeon Cho](mailto:tre0430@korea.ac.kr)

</div>
