<div align="center">

# FastDEM

<a href="https://github.com/Ikhyeon-Cho/FastDEM"><img src="https://img.shields.io/badge/C++17-00599C?logo=cplusplus&logoColor=white" /></a>
<a href="#start-with-ros1"><img src="https://img.shields.io/badge/ROS1-Noetic-blue" /></a>
<a href="#start-with-ros2"><img src="https://img.shields.io/badge/ROS2-Humble+-teal" /></a>
<a href=""><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
<br>
<a href="https://github.com/Ikhyeon-Cho/FastDEM/actions/workflows/build.yml"><img src="https://github.com/Ikhyeon-Cho/FastDEM/actions/workflows/build.yml/badge.svg" /></a>

> ***Ultra-fast 2.5D elevation mapping** on **embedded** CPUs — **100+ Hz** on Jetson Orin*

**[ROS1](#start-with-ros1)** · **[ROS2](#start-with-ros2)** · **[C++ Library](#use-fastdem-as-a-c-library)**

<br>

<p align="center">
  <img src="assets/fastdem_laser_local.gif" width="49%" alt="FastDEM real-time elevation mapping with LiDAR point cloud" />
  <img src="assets/fastdem_rgbd_local.gif" width="48.3%" alt="FastDEM elevation mapping with RGB-D camera" />
  <img src="assets/fastdem_rgbd_global.gif" width="97.5%" alt="FastDEM global elevation mapping in outdoor environment" />
</p>

</div>

FastDEM is an ultra-fast 2.5D elevation mapping library for mobile robots.
It builds dense elevation maps from LiDAR and RGB-D point clouds at 100+ Hz
on embedded CPUs (e.g. Jetson Orin), without GPU.

*Formerly known as `height_mapping`.*

---

<!-- ## Related Projects

**Traversability Analysis**
- **[LeSTA](https://github.com/Ikhyeon-Cho/LeSTA)** — Self-supervised traversability learning from robot navigation experiences (RA-L 2024)
- **[FastTrav](https://github.com/Ikhyeon-Cho/FastTrav)** — Classical online traversability analysis with geometry-based terrain features

**Point Cloud Library**
- **[nanoPCL](https://github.com/Ikhyeon-Cho/nanoPCL)** — Lightweight point cloud library for embedded systems

--- -->

## Features

* ***Fast*** — 100+ Hz on Jetson Orin. ~10ms per scan.
* ***Lightweight*** — Minimal dependencies. No OpenCV, PCL, or Open3D.
* ***ROS-agnostic*** — Clean C++ API, with optional ROS support.
* ***Sensor-Aware*** — Physics-based sensor models for LiDAR and RGB-D.
* ***Multiple Estimators*** — Kalman Filter (parametric), P² Quantile estimator (non-parametric).
* ***Local + Global Mapping*** — Robot-centric or map-centric terrain mapping.
* ***Post-processing*** — Raycasting, Uncertainty fusion, Inpainting, Feature extraction, etc.

---

## Performance

The mapping pipeline runs at **~10 ms** on embedded CPUs — fast enough to leave ample headroom for post-processing.

<p align="center">
  <img src="assets/fastdem_jetson_benchmark.svg" width="95%" alt="FastDEM benchmark on Jetson Orin" />
</p>

*Measured with Velodyne VLP-16 (~30K pts/scan) · 15×15 m map at 0.1 m resolution*

---

## Dependencies

- **Eigen3** — Linear algebra
- **yaml-cpp** — Configuration parsing
- **spdlog** — Logging

```bash
sudo apt install libeigen3-dev libyaml-cpp-dev libspdlog-dev
```

---

## Start with ROS1

**Prerequisites:** Ubuntu 20.04, [ROS Noetic](http://wiki.ros.org/noetic/Installation)

```bash
# ROS dependencies
sudo apt install ros-noetic-grid-map-msgs ros-noetic-tf2-eigen

# Clone and build
cd ~/catkin_ws/src
git clone https://github.com/Ikhyeon-Cho/FastDEM.git
catkin build fastdem_ros

# Run (add global_mapping:=true for map-centric mode)
roslaunch fastdem_ros run.launch rviz:=true
```

Configuration: [`ros1/config/local_mapping.yaml`](ros1/config/local_mapping.yaml) · [`ros1/config/global_mapping.yaml`](ros1/config/global_mapping.yaml)

---

## Start with ROS2

**Prerequisites:** Ubuntu 22.04, [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html)

*Other ROS2 distributions may also work but are not yet tested.*

```bash
# ROS dependencies
sudo apt install ros-humble-grid-map-msgs ros-humble-tf2-eigen

# Clone and build
cd ~/ros2_ws/src
git clone https://github.com/Ikhyeon-Cho/FastDEM.git
colcon build --packages-up-to fastdem_ros

# Run (add global_mapping:=true for map-centric mode)
ros2 launch fastdem_ros run.launch.py rviz:=true
```

Configuration: [`ros2/config/local_mapping.yaml`](ros2/config/local_mapping.yaml) · [`ros2/config/global_mapping.yaml`](ros2/config/global_mapping.yaml)

---

## Use FastDEM as a C++ Library

FastDEM can be used without ROS as a standalone C++ library.

```bash
git clone https://github.com/Ikhyeon-Cho/FastDEM.git
cd FastDEM/fastdem
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

```cpp
#include <fastdem/fastdem.hpp>

fastdem::ElevationMap map;
map.setGeometry(15.0, 15.0, 0.1);  // width, height, resolution [m]

auto cfg = fastdem::loadConfig("config/default.yaml");
fastdem::FastDEM mapper(map, cfg);

// With explicit transforms
mapper.integrate(cloud, T_base_sensor, T_world_base);
```

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

**Contact:** [ikhyeon.c@gmail.com](mailto:ikhyeon.c@gmail.com)

BSD-3-Clause License © [Ikhyeon Cho](mailto:ikhyeon.c@gmail.com)

</div>
