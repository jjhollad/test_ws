# Odometry Calibration Guide

## Drive One Meter Script

This script drives the robot forward exactly 1 meter (or configurable distance) using encoder data, allowing you to measure the actual distance traveled and calibrate your odometry parameters.

## Usage

### Basic Usage (1 meter forward)
```bash
cd ~/test_ws
source install/setup.bash

# Make sure your robot is running (motor driver, etc.)
ros2 run generic_motor_driver drive_one_meter.py
```

### With Custom Parameters
```bash
ros2 run generic_motor_driver drive_one_meter.py \
  --ros-args \
  -p wheel_radius:=0.12 \
  -p wheel_base:=0.57 \
  -p motor_gear_ratio:=90.0 \
  -p belt_drive_ratio:=6.4 \
  -p apply_gear_reduction:=true \
  -p encoder_reduction_factor:=10.75 \
  -p target_distance:=1.0 \
  -p speed:=0.2 \
  -p joint_names:="['left_rear_wheel_joint', 'right_rear_wheel_joint']"
```

### Custom Distance
```bash
# Drive 2 meters
ros2 run generic_motor_driver drive_one_meter.py --ros-args -p target_distance:=2.0

# Drive 0.5 meters
ros2 run generic_motor_driver drive_one_meter.py --ros-args -p target_distance:=0.5
```

### Custom Speed
```bash
# Drive slower (0.1 m/s)
ros2 run generic_motor_driver drive_one_meter.py --ros-args -p speed:=0.1

# Drive faster (0.3 m/s)
ros2 run generic_motor_driver drive_one_meter.py --ros-args -p speed:=0.3
```

## Calibration Procedure

1. **Mark starting position**: Place a marker (tape, object) at the robot's front edge
2. **Run the script**: Execute the drive_one_meter.py script
3. **Mark ending position**: When robot stops, mark where the front edge is
4. **Measure actual distance**: Measure the distance between the two marks
5. **Compare**: 
   - Script reports: "Distance traveled (encoder): X.XXXX meters"
   - Actual measurement: Y.YYYY meters
   - Ratio: actual / encoder = calibration factor

## Calculating Calibration Factor

If the script says the robot traveled 1.0500 meters but you measured 1.0000 meters:
- **Calibration factor** = 1.0000 / 1.0500 = 0.9524
- This means your encoder is reading 5% too high

## Adjusting Parameters

### If Encoder Distance > Actual Distance

The encoder is reading too high. Adjust:

1. **Increase `encoder_reduction_factor`**:
   - Current: 10.75
   - New: 10.75 * (actual / encoder) = 10.75 * 0.9524 = 10.24

2. **Or check `wheel_radius`**:
   - If wheel_radius is too large, distance will be overestimated
   - Measure actual wheel radius and update

### If Encoder Distance < Actual Distance

The encoder is reading too low. Adjust:

1. **Decrease `encoder_reduction_factor`**:
   - Current: 10.75
   - New: 10.75 * (actual / encoder) = 10.75 * 1.05 = 11.29

2. **Or check `wheel_radius`**:
   - If wheel_radius is too small, distance will be underestimated
   - Measure actual wheel radius and update

## Example Calibration Session

```bash
# 1. Start robot
ros2 launch generic_motor_driver complete_robot.launch.py

# 2. In another terminal, run calibration
ros2 run generic_motor_driver drive_one_meter.py

# Output:
# Distance traveled (encoder): 1.0500 meters
# Time elapsed: 5.25 seconds

# 3. Measure actual distance: 1.000 meters

# 4. Calculate correction:
# Ratio = 1.000 / 1.050 = 0.9524
# New encoder_reduction_factor = 10.75 * 0.9524 = 10.24

# 5. Update launch file with new value:
# encoder_reduction_factor:=10.24

# 6. Test again to verify
ros2 run generic_motor_driver drive_one_meter.py
```

## Multiple Test Runs

For best accuracy, run multiple tests:

1. **Test 1**: Drive 1 meter, measure, record
2. **Test 2**: Drive 1 meter, measure, record  
3. **Test 3**: Drive 1 meter, measure, record
4. **Average**: Calculate average ratio
5. **Apply**: Use average ratio to adjust parameters

## Troubleshooting

### Robot doesn't move
- Check that motor driver is running: `ros2 node list | grep motor`
- Check cmd_vel topic: `ros2 topic echo /cmd_vel`
- Check for errors in motor driver logs

### Distance calculation seems wrong
- Verify joint names match your URDF
- Check that encoder_reduction_factor matches your launch file
- Ensure wheel_radius and wheel_base are correct

### Robot drifts left/right
- This is normal for differential drive
- The script uses average of left and right wheels
- For straight-line calibration, ensure robot starts straight
- Consider using a guide rail or wall for very precise tests

## Advanced: Rotation Calibration

To calibrate rotation separately:

1. Mark robot's orientation (use a line on the floor)
2. Command rotation: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}"`
3. Rotate exactly 360 degrees (or 2π radians)
4. Compare encoder rotation vs actual rotation
5. Adjust `rotation_scale` parameter if needed

## Integration with Launch File

After calibration, update your launch file:

```python
encoder_reduction_factor_arg = DeclareLaunchArgument(
    'encoder_reduction_factor',
    default_value='10.24',  # Your calibrated value
    description='Calibrated encoder reduction factor'
)
```






