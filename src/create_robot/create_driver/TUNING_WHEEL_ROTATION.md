# Tuning Wheel Rotation in RViz

This guide explains how to synchronize the wheel rotation in RViz with the actual robot wheels.

## Parameters

The wheel rotation direction in RViz is controlled by encoder inversion parameters:

- `invert_left_encoder` (default: `false`) - Inverts left encoder direction
- `invert_right_encoder` (default: `true`) - Inverts right encoder direction

## Tuning Steps

### 1. Test Forward Motion

Send a forward command to the robot:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**Observe:**
- Real robot: Both wheels should rotate forward (counter-clockwise when viewed from left side, clockwise from right side)
- RViz: Both wheels should rotate in the same direction as the real robot

### 2. Adjust Left Wheel

If the left wheel in RViz rotates in the wrong direction:
```bash
ros2 launch generic_motor_driver complete_robot.launch.py invert_left_encoder:=true
```

Or if it's already `true`, set it to `false`:
```bash
ros2 launch generic_motor_driver complete_robot.launch.py invert_left_encoder:=false
```

### 3. Adjust Right Wheel

If the right wheel in RViz rotates in the wrong direction:
```bash
ros2 launch generic_motor_driver complete_robot.launch.py invert_right_encoder:=true
```

Or if it's already `true`, set it to `false`:
```bash
ros2 launch generic_motor_driver complete_robot.launch.py invert_right_encoder:=false
```

### 4. Test Rotation

Send a rotation command:
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

**Observe:**
- Real robot: Wheels should rotate in opposite directions (one forward, one backward)
- RViz: Should match the real robot

### 5. Final Verification

Drive the robot forward and verify:
- RViz wheels rotate in the same direction as real wheels
- Robot moves forward in RViz when real robot moves forward
- Robot rotates correctly in RViz when real robot rotates

## Quick Tuning Command

To test different combinations quickly:

```bash
# Test all combinations
ros2 launch generic_motor_driver complete_robot.launch.py \
  invert_left_encoder:=false \
  invert_right_encoder:=false

# Or
ros2 launch generic_motor_driver complete_robot.launch.py \
  invert_left_encoder:=true \
  invert_right_encoder:=true

# Or mixed
ros2 launch generic_motor_driver complete_robot.launch.py \
  invert_left_encoder:=false \
  invert_right_encoder:=true
```

## Permanent Configuration

Once you find the correct settings, update the launch file defaults:

Edit `complete_robot.launch.py`:
```python
invert_left_encoder_arg = DeclareLaunchArgument(
    'invert_left_encoder',
    default_value='true',  # Change to your tuned value
    ...
)

invert_right_encoder_arg = DeclareLaunchArgument(
    'invert_right_encoder',
    default_value='false',  # Change to your tuned value
    ...
)
```

## Tuning Gear Ratio (Wheel Speed Mismatch)

If the real robot wheels are turning much slower or faster than RViz, the gear ratio is incorrect.

### Current Calculation

The joint position is calculated as:
```
wheel_rotations = encoder_counts / (motor_gear_ratio * belt_drive_ratio)
```

### How to Tune

1. **Measure the actual ratio:**
   - Drive the robot forward a known distance (e.g., 1 meter)
   - Check the encoder counts from the motor controller
   - Calculate: `actual_reduction = encoder_counts / wheel_rotations`
   - Wheel rotations = distance / (2 * π * wheel_radius)

2. **Compare with current settings:**
   ```bash
   # Check current values in launch file or when running:
   ros2 param get /generic_motor_driver motor_gear_ratio
   ros2 param get /generic_motor_driver belt_drive_ratio
   ```

3. **Adjust the gear ratio:**
   ```bash
   # If wheels are slower in reality, INCREASE the gear ratio
   ros2 launch generic_motor_driver complete_robot.launch.py \
     motor_gear_ratio:=210.0 \
     belt_drive_ratio:=6.4
   
   # For rectangular robot (default should be):
   # motor_gear_ratio: 210.0
   # belt_drive_ratio: 6.4
   # Total reduction: 210.0 * 6.4 = 1344.0
   ```

4. **Quick test method:**
   - Mark a point on the real wheel
   - Send a forward command for 1 second
   - Count how many times the wheel rotates in reality
   - Compare with RViz wheel rotations
   - Adjust gear ratio proportionally:
     ```
     new_gear_ratio = old_gear_ratio * (rviz_rotations / real_rotations)
     ```

### Example

If RViz shows 10 rotations but real wheel only does 5:
- Current total reduction: 576.0 (90.0 * 6.4)
- New total reduction needed: 576.0 * (10/5) = 1152.0
- If keeping belt_drive_ratio at 6.4: motor_gear_ratio = 1152.0 / 6.4 = 180.0

## Troubleshooting

**Wheels spin backwards in RViz:**
- Toggle the `invert_left_encoder` or `invert_right_encoder` parameter

**Wheels don't rotate at all in RViz:**
- Check that joint states are being published: `ros2 topic echo /joint_states`
- Verify joint names match URDF: `left_rear_wheel_joint` and `right_rear_wheel_joint`

**Wheels rotate but robot doesn't move:**
- Check odometry: `ros2 topic echo /odom`
- Verify TF transforms: `ros2 run tf2_ros tf2_echo odom base_footprint`

**Real wheels much slower than RViz:**
- Increase `motor_gear_ratio` or `belt_drive_ratio`
- The total reduction ratio (motor_gear_ratio * belt_drive_ratio) should be larger

**Real wheels much faster than RViz:**
- Decrease `motor_gear_ratio` or `belt_drive_ratio`
- The total reduction ratio should be smaller

