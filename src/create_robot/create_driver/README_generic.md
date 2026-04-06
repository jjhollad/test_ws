# Generic Motor Driver

A ROS2 driver for serial-based motor controllers, adapted from the Create robot driver. This driver provides a generic interface for controlling 4-channel serial motor controllers and 4-channel relay modules.

## Features

### Motor Controller
- **Serial Communication**: Communicates with motor controllers via serial port
- **Odometry**: Calculates and publishes robot odometry from wheel encoders
- **Joint States**: Publishes joint states for all 4 motors
- **Velocity Control**: Accepts `cmd_vel` commands and converts them to motor speeds
- **Gear Ratio Support**: Accounts for motor gear ratios and belt drive ratios
- **Diagnostics**: Provides diagnostic information about serial connection and driver performance
- **TF Broadcasting**: Publishes odometry transforms

### Relay Controller
- **4-Channel Relay Control**: Controls up to 4 relays via Arduino DUE
- **Individual Relay Control**: Control each relay independently
- **Bulk Relay Control**: Control all relays simultaneously
- **Status Monitoring**: Real-time relay status feedback
- **Serial Communication**: Communicates with Arduino DUE via serial port

## Supported Motor Controller Protocol

This driver is designed for motor controllers that use the following protocol:

### Motor Control Commands
- `$spd:M1,M2,M3,M4#` - Set motor speeds (M1-M4: -100 to 100)

### Data Requests
- `$upload:0,0,0#` - Request encoder and speed data

### Data Responses
- `$MAll:M1,M2,M3,M4#` - Total encoder counts for all motors
- `$MTEP:M1,M2,M3,M4#` - Real-time encoder data (10ms)
- `$MSPD:M1,M2,M3,M4#` - Current motor speeds

## Installation

1. Copy the files to your ROS2 workspace
2. Rename `package_generic.xml` to `package.xml`
3. Rename `CMakeLists_generic.txt` to `CMakeLists.txt`
4. Build the package:
   ```bash
   colcon build --packages-select generic_motor_driver
   ```

## Usage

### Motor Controller Launch
```bash
# Basic launch
ros2 launch generic_motor_driver generic_motor_driver.launch.py

# With custom parameters
ros2 launch generic_motor_driver generic_motor_driver.launch.py \
  dev:=/dev/ttyUSB0 \
  baud:=115200 \
  wheel_base:=0.3 \
  wheel_radius:=0.05 \
  motor_gear_ratio:=210.0 \
  belt_drive_ratio:=6.4
```

### Relay Controller Launch
```bash
# Launch relay controller
ros2 launch generic_motor_driver relay_controller.launch.py dev:=/dev/ACM0

# With custom parameters
ros2 launch generic_motor_driver relay_controller.launch.py \
  dev:=/dev/ttyACM0 \
  baud:=115200 \
  status_publish_rate:=2.0
```

### Complete System Launch
```bash
# Launch both motor controller and robot visualization
ros2 launch generic_motor_driver complete_robot.launch.py
```

## Parameters

### Motor Controller Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `dev` | `/dev/ttyUSB0` | Serial device path |
| `baud` | `115200` | Serial baud rate |
| `wheel_base` | `0.3` | Distance between left and right wheels (meters) |
| `wheel_radius` | `0.05` | Wheel radius (meters) |
| `motor_gear_ratio` | `1.0` | Motor internal gear ratio (e.g., 210.0 for 210:1) |
| `belt_drive_ratio` | `1.0` | Belt drive ratio (e.g., 6.4 for 6.4:1) |
| `base_frame` | `base_footprint` | Base frame ID |
| `odom_frame` | `odom` | Odometry frame ID |
| `loop_hz` | `10.0` | Control loop frequency (Hz) |
| `publish_tf` | `true` | Whether to publish TF transforms |
| `latch_cmd_duration` | `0.2` | Duration to latch velocity commands (seconds) |

### Relay Controller Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `dev` | `/dev/ttyUSB1` | Serial device path for Arduino DUE |
| `baud` | `115200` | Serial baud rate |
| `status_publish_rate` | `1.0` | Status publishing rate (Hz) |

## Topics

### Motor Controller Topics

#### Subscribed Topics
- `/cmd_vel` (`geometry_msgs/msg/Twist`) - Velocity commands

#### Published Topics
- `/odom` (`nav_msgs/msg/Odometry`) - Robot odometry
- `/joint_states` (`sensor_msgs/msg/JointState`) - Joint states for all motors
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`) - Diagnostic information

### Relay Controller Topics

#### Subscribed Topics
- `/relay1` (`std_msgs/msg/Bool`) - Control relay 1 (true=ON, false=OFF)
- `/relay2` (`std_msgs/msg/Bool`) - Control relay 2 (true=ON, false=OFF)
- `/relay3` (`std_msgs/msg/Bool`) - Control relay 3 (true=ON, false=OFF)
- `/relay4` (`std_msgs/msg/Bool`) - Control relay 4 (true=ON, false=OFF)
- `/relay_all` (`std_msgs/msg/UInt8MultiArray`) - Control all relays [relay1, relay2, relay3, relay4]
- `/relay_command` (`std_msgs/msg/String`) - Send custom commands to Arduino

#### Published Topics
- `/relay_status` (`std_msgs/msg/UInt8MultiArray`) - Current relay states [0/1, 0/1, 0/1, 0/1]
- `/relay_feedback` (`std_msgs/msg/String`) - Arduino response messages

## Relay Control Examples

### Individual Relay Control
```bash
# Turn on relay 1
ros2 topic pub /relay1 std_msgs/msg/Bool "data: true"

# Turn off relay 1
ros2 topic pub /relay1 std_msgs/msg/Bool "data: false"

# Turn on relay 2
ros2 topic pub /relay2 std_msgs/msg/Bool "data: true"
```

### Bulk Relay Control
```bash
# Control all relays at once [relay1, relay2, relay3, relay4]
ros2 topic pub /relay_all std_msgs/msg/UInt8MultiArray "data: [1, 0, 1, 0]"

# Turn all relays on
ros2 topic pub /relay_all std_msgs/msg/UInt8MultiArray "data: [1, 1, 1, 1]"

# Turn all relays off
ros2 topic pub /relay_all std_msgs/msg/UInt8MultiArray "data: [0, 0, 0, 0]"
```

### Custom Commands
```bash
# Send custom commands to Arduino
ros2 topic pub /relay_command std_msgs/msg/String "data: 'ALL_ON'"
ros2 topic pub /relay_command std_msgs/msg/String "data: 'ALL_OFF'"
ros2 topic pub /relay_command std_msgs/msg/String "data: 'STATUS'"
```

### Monitor Relay Status
```bash
# Check relay status
ros2 topic echo /relay_status

# Check Arduino feedback
ros2 topic echo /relay_feedback
```

## Arduino DUE Setup

### Wiring
Connect the 4-channel relay module to your Arduino DUE:

```
Relay Module    →    Arduino DUE
─────────────────────────────────
GND            →    GND
VCC            →    5V (or 3.3V depending on relay module)
IN1            →    Digital Pin 2
IN2            →    Digital Pin 3
IN3            →    Digital Pin 4
IN4            →    Digital Pin 5
```

### Arduino Sketch
Upload the provided `relay_controller.ino` sketch to your Arduino DUE. The sketch:
- Listens for serial commands from ROS2
- Controls 4 relays based on commands
- Sends status feedback back to ROS2
- Handles command concatenation issues

### Supported Arduino Commands
- `RELAY1_ON`, `RELAY1_OFF` - Control relay 1
- `RELAY2_ON`, `RELAY2_OFF` - Control relay 2
- `RELAY3_ON`, `RELAY3_OFF` - Control relay 3
- `RELAY4_ON`, `RELAY4_OFF` - Control relay 4
- `ALL_ON`, `ALL_OFF` - Control all relays
- `STATUS` - Get current relay status

## Gear Ratio Configuration

### Understanding Gear Ratios
The driver accounts for both motor gear ratios and belt drive ratios:

- **Motor Gear Ratio**: Internal reduction in the motor (e.g., 210.0 for 210:1)
- **Belt Drive Ratio**: Pulley ratio between motor and wheel (e.g., 6.4 for 6.4:1)
- **Total Reduction**: Motor gear ratio × Belt drive ratio

### Example Configurations

#### High Torque Setup (210:1 gear motor + 6.4:1 belt drive):
```bash
ros2 launch generic_motor_driver generic_motor_driver.launch.py \
  motor_gear_ratio:=210.0 \
  belt_drive_ratio:=6.4 \
  wheel_radius:=0.075
```

#### Direct Drive Setup (no gear reduction):
```bash
ros2 launch generic_motor_driver generic_motor_driver.launch.py \
  motor_gear_ratio:=1.0 \
  belt_drive_ratio:=1.0 \
  wheel_radius:=0.05
```

### Hard-coding Gear Ratios
To hard-code gear ratios in the driver, modify the constructor in `generic_motor_driver.cpp`:

```cpp
motor_gear_ratio_ = 210.0;  // Your motor gear ratio
belt_drive_ratio_ = 6.4;   // Your belt drive ratio
total_reduction_ratio_ = motor_gear_ratio_ * belt_drive_ratio_;
```

## Robot Configuration

### Differential Drive Setup
For a differential drive robot:
- Motor 1: Left wheel
- Motor 2: Right wheel
- Motors 3-4: Not used (set to 0)

### 4-Wheel Drive Setup
For a 4-wheel drive robot:
- Motor 1: Front left
- Motor 2: Front right
- Motor 3: Rear left
- Motor 4: Rear right

## Customization

### Adding Sensor Support
To add support for additional sensors provided by your motor controller:

1. Add sensor data parsing in `parseMotorData()`
2. Add corresponding publishers in the constructor
3. Add sensor data publishing in the update loop

### Modifying Motor Control
To change how velocity commands are converted to motor speeds:

1. Modify the `cmdVelCallback()` function
2. Adjust the `sendMotorSpeeds()` function if needed
3. Update the odometry calculation in `updateOdometry()` if using different wheel configuration

## Troubleshooting

### Serial Connection Issues
- Check that the device path is correct (`/dev/ttyUSB0`, `/dev/ttyACM0`, etc.)
- Verify the baud rate matches your motor controller/Arduino
- Ensure the user has permission to access the serial device:
  ```bash
  sudo usermod -a -G dialout $USER
  ```

### Motor Controller Issues

#### No Odometry Data
- Verify that your motor controller is sending encoder data
- Check that the `$upload:1,1,1#` command is being sent periodically
- Monitor the serial communication with:
  ```bash
  ros2 topic echo /joint_states
  ```

#### Motor Not Moving
- Check that velocity commands are being received:
  ```bash
  ros2 topic echo /cmd_vel
  ```
- Verify that motor speed commands are being sent to the controller
- Check the motor controller's status LEDs or indicators

#### Incorrect Odometry
- Verify gear ratios are set correctly
- Check wheel radius and wheel base measurements
- Test with known distances to calibrate

### Relay Controller Issues

#### Relays Not Responding
- Check Arduino DUE is connected and powered
- Verify relay module wiring (GND, VCC, IN1-4)
- Check relay module voltage (5V vs 3.3V)
- Monitor Arduino feedback:
  ```bash
  ros2 topic echo /relay_feedback
  ```

#### Command Concatenation Errors
- The driver includes delays to prevent command concatenation
- Arduino sketch handles concatenated commands automatically
- If issues persist, check serial communication timing

#### Relay Status Not Updating
- Check relay status topic:
  ```bash
  ros2 topic echo /relay_status
  ```
- Verify Arduino is sending STATUS responses
- Check relay module power and connections

## License

BSD License - see LICENSE file for details.


Teleop with contoller launch command
ros2 launch teleop_twist_joy teleop-launch.py joy_config:=xbox cmd_vel:=/cmd_vel published_stamped_twist:=true

