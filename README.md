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
- C++ compiler with C++17 support

### Building
```bash
cd ~/ros2_ws/src
git clone https://github.com/rodep-soft/ddsm115_ros2_driver.git
cd ~/ros2_ws
colcon build --packages-select ddsm115_ros2_driver
source install/setup.bash
```

## Usage

### Running the driver node
```bash
ros2 launch ddsm115_ros2_driver ddsm115.launch.py
```

### With custom parameters
```bash
ros2 launch ddsm115_ros2_driver ddsm115.launch.py serial_port:=/dev/ttyUSB1 motor_id:=2
```

### Running without launch file
```bash
ros2 run ddsm115_ros2_driver ddsm115_node
```

## Parameters
- `serial_port` (string, default: "/dev/ttyUSB0"): Serial port for communication
- `baud_rate` (int, default: 115200): Baud rate for serial communication
- `motor_id` (int, default: 1): Motor ID for the controller
- `publish_rate` (double, default: 10.0): Rate at which to publish joint states (Hz)

## Published Topics
- `/joint_states` (sensor_msgs/JointState): Joint state information
- `/motor_status` (std_msgs/String): Motor status messages

## License
MIT License - see LICENSE file for details
