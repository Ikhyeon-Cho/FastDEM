<div align="center">
  <h1>
    <img src="assets/fastdem.svg" align="center" height="46" alt="">
    FastDEM
  </h1>
</div>

<p align="center"><i><b>Ultra-fast elevation mapping</b> on <b>embedded</b> robots — <b>100+ Hz</b> on Jetson Orin</i></p>

<p align="center">
  <a href="#start-with-ros1">ROS1</a> ·
  <a href="#start-with-ros2">ROS2</a> ·
  <a href="#use-fastdem-as-a-c-library">C++ API</a>
</p>

<p align="center">
  <img src="assets/fastdem_lidar.gif" width="49%" alt="FastDEM real-time elevation mapping with LiDAR point cloud" />
  <img src="assets/fastdem_rgbd.gif" width="48.3%" alt="FastDEM elevation mapping with RGB-D camera" />
  <img src="assets/fastdem_global.gif" width="97.5%" alt="FastDEM global elevation mapping in outdoor environment" />
</p>

**FastDEM** is a **lightweight elevation mapping engine** for mobile robots that need dense terrain maps at control-loop rates. It incrementally fuses LiDAR or RGB-D point clouds into local or global elevation maps. The library runs entirely on CPU, and supports terrain post-processing for downstream navigation and traversability analysis.

**Related projects:**
- **Terrablox** — (in preparation)
- **[LeSTA](https://github.com/Ikhyeon-Cho/LeSTA)** — Self-supervised traversability learning for mobile robots (*RA-L 2024*)

---

## Features

* **Fast** — 100+ Hz on Jetson Orin. ~10ms per scan, on CPU alone
* **Lightweight** — No <i>PCL, OpenCV, or CUDA required</i>. Just Eigen at core.
* **ROS-agnostic** — Clean C++ API, with optional ROS adapters
* **Sensor-Aware** — Physics-based sensor models for LiDAR and RGB-D range measurements.
* **Multiple Estimators** — Kalman Filter (parametric), P² Quantile estimator (non-parametric).
* **Local + Global Mapping** — Support both robot-centric / map-centric implementation
* **Post-processing** — Raycasting, Uncertainty fusion, Inpainting, Feature extraction, and more.

---

## Performance

The mapping pipeline runs at **~10 ms** on embedded CPUs — fast enough to leave ample room for post-processing.

<p align="center">
  <img src="assets/fastdem_benchmark.svg" width="95%" alt="FastDEM benchmark on Jetson Orin" />
</p>

*Measured with Velodyne VLP-16 (~30K pts/scan) · 15×15 m map at 0.1 m resolution*

---

## Dependencies

- **Eigen3**, **yaml-cpp**, **spdlog**
- **[nanoGrid](https://github.com/Ikhyeon-Cho/nanoGrid)**, **nanoPCL** — bundled automatically

---

## Start with ROS1

**Prerequisites:** Ubuntu 20.04, [ROS Noetic](http://wiki.ros.org/noetic/Installation)

```bash
# Dependencies
sudo apt install libeigen3-dev libyaml-cpp-dev libspdlog-dev
sudo apt install ros-noetic-tf2-eigen ros-noetic-grid-map-msgs

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

> Other ROS2 distributions may also work but are not yet tested.

```bash
# Dependencies
sudo apt install libeigen3-dev libyaml-cpp-dev libspdlog-dev
sudo apt install ros-humble-tf2-eigen ros-humble-grid-map-msgs

# Clone and build
cd ~/ros2_ws/src
git clone https://github.com/Ikhyeon-Cho/FastDEM.git
colcon build --packages-up-to fastdem_ros2

# Run (add global_mapping:=true for map-centric mode)
ros2 launch fastdem_ros2 run.launch.py rviz:=true
```

Configuration: [`ros2/config/local_mapping.yaml`](ros2/config/local_mapping.yaml) · [`ros2/config/global_mapping.yaml`](ros2/config/global_mapping.yaml)

---

## Use FastDEM as a C++ Library

FastDEM can be used without ROS as a standalone C++ core library.

```bash
# Dependencies
sudo apt install libeigen3-dev libyaml-cpp-dev libspdlog-dev

# Clone and build
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
