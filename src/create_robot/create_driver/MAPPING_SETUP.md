# Mapping Setup for Custom Robot

This document explains the changes needed to adapt `sweepMapping_launch.py` for your custom robot.

## Key Differences Between sweepMapping_launch.py and Your Custom Robot

### 1. **Robot Bringup**
- **sweepMapping_launch.py**: Uses `create_bringup create_2.launch` (iRobot Create robot)
- **Your robot**: Uses custom `generic_motor_driver` and `relay_controller` nodes

### 2. **Lidar Setup**
- **sweepMapping_launch.py**: Uses `sensors_launch.py` which:
  - Publishes a static transform from `base_footprint` to `laser_frame`
  - Uses `rplidar_composition` node
- **Your robot**: 
  - URDF already defines `laser_frame` (no static transform needed)
  - Uses `rplidar_node` directly

### 3. **SLAM Tool**
- **sweepMapping_launch.py**: Uses **Cartographer** (`cartographer.launch.py`)
- **Your robot**: Uses **SLAM Toolbox** (more modern, easier to configure)

### 4. **Frame Names**
Both use the same frame names:
- `base_footprint` - base frame
- `odom` - odometry frame  
- `map` - map frame
- `laser_frame` - lidar frame

## What Was Created

### 1. `complete_robot_mapping.launch.py`
A new launch file that combines:
- Your custom robot nodes (motor driver, relay controller, lidar)
- SLAM Toolbox for mapping
- Robot state publisher
- RViz visualization

### 2. `config/mapper_params_online_async.yaml`
SLAM Toolbox configuration file with:
- Frame names matching your robot (`base_footprint`, `odom`, `map`)
- Scan topic: `/scan`
- Mapping mode enabled
- Optimized parameters for real-time mapping

## How to Use

### Start Mapping:
```bash
cd ~/test_ws
source install/setup.bash
ros2 launch create_driver complete_robot_mapping.launch.py
```

### Save the Map:
Once you've mapped your environment, save it:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/test_ws/map/my_map
```

### Use Saved Map for Localization:
To use a saved map instead of mapping, edit `mapper_params_online_async.yaml`:
```yaml
mode: localization  # Change from 'mapping'
map_file_name: my_map  # Uncomment and set your map name
```

## Optional: Add Navigation (Nav2)

If you want to add navigation capabilities like in sweepMapping_launch.py:

1. **Create nav2_params.yaml** (similar to sweepbot's config)
2. **Add Nav2 launch** to your mapping launch file:
```python
nav2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('nav2_bringup'),
            'launch',
            'navigation_launch.py'
        ])
    ]),
    launch_arguments={
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'params_file': PathJoinSubstitution([
            FindPackageShare('create_driver'),
            'config',
            'nav2_params.yaml'
        ])
    }.items()
)
```

## Key Configuration Points

### Frame Transform Chain
Your robot should have this TF chain:
```
map -> odom -> base_footprint -> base_link -> laser_frame
```

- `map -> odom`: Published by SLAM Toolbox
- `odom -> base_footprint`: Published by `generic_motor_driver`
- `base_footprint -> base_link`: Defined in URDF
- `base_link -> laser_frame`: Defined in URDF

### Important Parameters to Adjust

In `mapper_params_online_async.yaml`:
- `max_laser_range`: Set to your lidar's max range (default: 8.0m)
- `resolution`: Map resolution in meters (default: 0.05m = 5cm)
- `minimum_travel_distance`: Minimum distance before updating map (default: 0.25m)
- `minimum_travel_heading`: Minimum rotation before updating map (default: 0.5 rad)

## Troubleshooting

### No map being published
- Check that `/scan` topic is publishing: `ros2 topic echo /scan`
- Verify TF chain: `ros2 run tf2_tools view_frames`
- Check SLAM Toolbox logs for errors

### Map looks wrong
- Verify `base_frame` matches your robot's base frame (`base_footprint`)
- Check that `scan_topic` matches your lidar topic (`/scan`)
- Ensure lidar frame (`laser_frame`) is correctly defined in URDF

### Robot not moving in map
- Verify odometry is publishing: `ros2 topic echo /odom`
- Check TF from `odom` to `base_footprint` is being published






