# ddsm115_ros2_driver
ROS2 driver package for DDSM115 servo motor controller

## Overview
This package provides a ROS2 driver for the DDSM115 servo motor controller, compatible with ROS2 Kilted and other distributions.

## Features
- Serial communication with DDSM115 controller
- Joint state publishing
- Configurable motor parameters
- Launch file support

## Installation

### Prerequisites
- ROS2 (Kilted or compatible distribution)
- C++ compiler with C++20 support

### Building
```bash
cd ~/ros2_ws/src
git clone https://github.com/rodep-soft/ddsm115_ros2_driver.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select ddsm115_ros2_driver
source install/setup.bash
```

## Usage

### Running the driver node
```bash
```

### With custom parameters
```bash
```

### Running without launch file
```bash
ros2 run ddsm115_ros2_driver ddsm115_ros2_node
```

## Parameters
- `serial_port` (string, default: "/dev/ttyUSB0"): Serial port for communication
- `motor_ids` (int[], default: [1]): Motor ID for the controller
- `publish_rate` (double, default: 10.0): Rate at which to publish joint states (Hz)

## Published Topics
- `/motor_[id]/command` (ddsm115_ros2_driver/msg/ddsm115_command): motor command
- `/motor_status` (ddsm115_ros2_driver/msg/ddsm115_status): Motor status 

## License
MIT License - see LICENSE file for details
