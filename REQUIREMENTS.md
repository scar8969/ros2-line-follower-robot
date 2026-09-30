# Project Requirements

## System Specifications
* **Operating System:** Ubuntu 24.04 LTS (for the Gazebo simulation)
* **Middleware:** ROS 2 Jazzy Jalisco
* **Terminal Emulator:** Tilix (Required for Make automation)
* **Simulation Environment:** Gazebo Harmonic (Default for ROS 2 Jazzy)

> **Headless option:** the `scripts/` test suite and the 2D simulator
> (`python3 -m line_follower_az.simulator --laps 3`) run on **any OS** with
> just Python 3 + numpy + matplotlib — no ROS2, no Gazebo. This is how the
> control pipeline is verified on Windows/macOS.

## Python Dependencies
The ROS 2 nodes utilize standard ROS 2 libraries. Ensure you have the following
packages installed:
* `rclpy`
* `geometry_msgs`
* `sensor_msgs`
* `std_msgs`
* `setuptools`

For the headless simulator and tests (any OS):
* `numpy`
* `matplotlib`

## Installation
To install the system-level dependencies required for the Gazebo simulation:

```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz tilix rqt ros-jazzy-rqt-reconfigure
rosdep update
rosdep install --from-paths src -y --ignore-src
```

For the headless simulator only:

```bash
pip install numpy matplotlib
```
