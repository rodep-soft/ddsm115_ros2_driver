# ddsm115_ros2_driver

ROS 2 driver package for the DDSM115 direct drive servo motor.

## Overview

This package provides a ROS 2 driver for DDSM115 motors, enabling control via serial communication (RS485). It supports multiple control modes (current, velocity, position) and can manage multiple motors on the same bus.

## Features

- **Multiple Control Modes**:
  - Current (Torque) Control
  - Velocity Control (RPM)
  - Position Control (0-360 degrees)
- **Multi-Motor Support**: Control multiple motors using their unique IDs.
- **Real-time Feedback**: Publishes current, velocity, and position feedback from each motor.
- **Utility Scripts**: Includes a script to set motor IDs.

## Installation

### Prerequisites

- ROS 2 (Humble, Iron, Jazzy, or Kilted)
- `boost` library
- `serial` (python3-serial) for the utility script

### Building

```bash
cd ~/ros2_ws/src
git clone https://github.com/rodep-soft/ddsm115_ros2_driver.git
cd ~/ros2_ws
colcon build --symlink-install --packages-select ddsm115_ros2_driver
source install/setup.bash
```

## Usage

### 1. Hardware Setup

Connect your DDSM115 motors to your computer using a USB-to-RS485 adapter. Ensure you have permissions to access the serial port:

```bash
sudo usermod -a -G dialout $USER
# Re-login for changes to take effect
```

### 2. Setting Motor ID

If you have multiple motors, each must have a unique ID (1-253). Use the provided script to set the ID:

```bash
# Set motor ID to 2 on /dev/ttyUSB0
ros2 run ddsm115_ros2_driver set_motor_id.py --ros-args -p serial_port:=/dev/ttyUSB0 -p id:=2
```
*Note: You must power-cycle the motor after setting the ID for changes to take effect.*

### 3. Running the Driver

Launch the driver node with the default configuration:

```bash
ros2 launch ddsm115_ros2_driver ddsm115.launch.py
```

To use a custom parameter file:

```bash
ros2 launch ddsm115_ros2_driver ddsm115.launch.py params_file:=/path/to/your/config.yaml
```

## ROS Interface

### Published Topics

- `motor_[id]/status` ([ddsm115_ros2_driver/msg/Ddsm115Status](./msg/Ddsm115Status.msg))
  - Feedback from the motor including current, velocity, position, and error codes.

### Subscribed Topics

- `motor_[id]/command` ([ddsm115_ros2_driver/msg/Ddsm115Command](./msg/Ddsm115Command.msg))
  - Send commands to the motor. Requires specifying the mode (current, velocity, or position).

### Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `serial_port` | string | `/dev/ttyUSB0` | Serial port device path. |
| `publish_rate` | double | `20.0` | Rate (Hz) at which status is published and commands are sent. |
| `motor_ids` | int_array | `[1, 2, 3, 4]` | List of motor IDs to manage. |

*Note: The baud rate is fixed at 115200 bps.*

## Message Definitions

### Ddsm115Command.msg

```msg
uint8 MODE_CURRENT = 1
uint8 MODE_VELOCITY = 2
uint8 MODE_POSITION = 3

uint8 BRAKE_RELEASE = 0
uint8 BRAKE_LOCK = 1

uint8 mode        # Control mode
float64 value     # Command value (A, RPM, or Degrees)
uint8 brake_mode  # Brake behavior (for velocity mode)
```

### Ddsm115Status.msg

```msg
float64 current   # Current (A)
float64 velocity  # Velocity (RPM)
float64 position  # Position (Degrees, 0-360)
uint8 error_code  # Error code from motor
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
