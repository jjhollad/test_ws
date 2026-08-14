# MPU-6050 Arduino Due Setup

This robot can publish MPU-6050 IMU data through the existing relay-controller Arduino Due serial connection. The Arduino firmware reads the MPU-6050 over I2C and sends lines like:

```text
IMU:ax,ay,az,gx,gy,gz
```

The ROS `relay_controller` node converts those lines to:

```text
/imu/data_raw  sensor_msgs/msg/Imu
```

Acceleration is published in `m/s^2`. Angular velocity is published in `rad/s`.

## Important Voltage Note

The Arduino Due is a **3.3 V board**. Its I/O pins are **not 5 V tolerant**.

Use an MPU-6050 breakout that is safe for 3.3 V I2C, or make sure the breakout's pullups are tied to 3.3 V, not 5 V.

## Wiring

Use the Arduino Due I2C pins near the communication header:

| MPU-6050 pin | Arduino Due pin | Notes |
| --- | --- | --- |
| VCC / VIN | 3.3V | Use 3.3 V unless your breakout explicitly requires VIN. |
| GND | GND | Common ground is required. |
| SDA | SDA / pin 20 | I2C data. |
| SCL | SCL / pin 21 | I2C clock. |
| AD0 | GND | Keeps address at `0x68`, which the firmware expects. |
| INT | Not connected | Not used by current firmware. |

Some MPU-6050 boards label the I2C pins as `SDA` and `SCL`; others may expose `XDA/XCL` too. Use `SDA/SCL`, not `XDA/XCL`.

## Mounting Orientation

Mount the IMU rigidly to the robot chassis. Do not leave it loose or attached only by wires.

The current URDF assumes:

| IMU axis | Robot direction |
| --- | --- |
| +X | Forward |
| +Y | Left |
| +Z | Up |

The ROS frame is:

```text
imu_link
```

If the physical board is rotated relative to the robot, update the `imu_joint` rotation in:

```text
create_description/urdf/rectangular_robot.urdf.xacro
```

## Firmware Behavior

On startup, the Arduino:

1. Initializes the relay pins.
2. Initializes the MPU-6050 at I2C address `0x68`.
3. Calibrates gyro bias while the robot is assumed stationary.
4. Prints one of:

```text
MPU6050_READY
MPU6050_NOT_FOUND
```

Keep the robot still during Arduino startup so the gyro bias calibration is valid.

The firmware publishes IMU data at 50 Hz. Relay control continues to use the same serial connection.

## Flashing

Open this sketch in the Arduino IDE:

```text
create_driver/arduino/relay_controller/relay_controller.ino
```

Select:

```text
Board: Arduino Due
Port: the relay Arduino serial port
```

Then upload.

No extra Arduino IMU library is required for the current firmware; it uses `Wire.h` directly.

## ROS Verification

After launching the robot stack, check that the IMU topic exists:

```bash
ros2 topic list | grep imu
```

Check update rate:

```bash
ros2 topic hz /imu/data_raw
```

Expected rate is about:

```text
50 Hz
```

Inspect one message:

```bash
ros2 topic echo /imu/data_raw --once
```

When the robot is sitting still:

- `linear_acceleration.z` should be near `+9.8` or `-9.8` depending on board orientation.
- `angular_velocity.z` should be near `0.0`.
- If `angular_velocity.z` is drifting while still, restart the Arduino while the robot is motionless.

## Current Limitations

The MPU-6050 does not provide absolute yaw. The current firmware publishes raw acceleration and gyro velocity only; orientation is marked unavailable in the ROS message.

Use this first to verify stable `/imu/data_raw`. After that, fuse it with wheel odometry using `robot_localization`.
