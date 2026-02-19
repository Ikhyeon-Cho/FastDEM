<div align="center">

# FastDEM

<a href="https://github.com/Ikhyeon-Cho/FastDEM/tree/main"><img src="https://img.shields.io/badge/C++17-00599C?logo=cplusplus&logoColor=white" /></a>
<a href="https://github.com/Ikhyeon-Cho/FastDEM/tree/ros1"><img src="https://img.shields.io/badge/ROS1-Noetic-blue" /></a>
<a href="https://github.com/Ikhyeon-Cho/FastDEM/tree/ros2"><img src="https://img.shields.io/badge/ROS2-Humble-lightgrey" /></a>
<a href=""><img src="https://img.shields.io/badge/Linux-FCC624?logo=linux&logoColor=black" /></a>
<br>
<a href="https://github.com/Ikhyeon-Cho/FastDEM/actions/workflows/build.yml"><img src="https://github.com/Ikhyeon-Cho/FastDEM/actions/workflows/build.yml/badge.svg?branch=ros1" /></a>

> ***Real-time elevation mapping** on **embedded** CPUs — **100+ Hz** on Jetson Orin*

**[Quick Start (ROS1)](#start-with-ros1)** · **[Core Library](https://github.com/Ikhyeon-Cho/FastDEM/tree/main)** · **[ROS2](https://github.com/Ikhyeon-Cho/FastDEM/tree/ros2)**

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

## Start with ROS1

### 1. Installation

**Prerequisites:** Ubuntu 20.04, [ROS Noetic](http://wiki.ros.org/noetic/Installation)

**Install dependencies**

```bash
sudo apt install libeigen3-dev libyaml-cpp-dev libspdlog-dev
sudo apt install ros-noetic-grid-map-msgs ros-noetic-tf2-eigen
```

**Clone and build**

```bash
cd ~/catkin_ws/src
git clone -b ros1 https://github.com/Ikhyeon-Cho/FastDEM.git
catkin build fastdem_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 2. Run

```bash
roslaunch fastdem_ros run.launch rviz:=true
```

For global (fixed-origin) mapping:

```bash
roslaunch fastdem_ros run.launch global_mapping:=true rviz:=true
```

### 3. Configuration

All default settings are in [`fastdem_ros/config/local_mapping.yaml`](fastdem_ros/config/local_mapping.yaml).

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
