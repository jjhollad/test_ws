# Nav2 Navigation Setup

Nav2 (Navigation2) is now integrated into your robot launch files, providing autonomous navigation capabilities.

## Launch Files

### 1. Mapping with Nav2 (Optional)
```bash
ros2 launch generic_motor_driver complete_robot_mapping.launch.py use_nav2:=true
```

This launches:
- Your robot (motor driver, lidar, etc.)
- SLAM Toolbox for mapping
- Nav2 navigation stack (optional, disabled by default)
- RViz

### 2. Navigation with Saved Map
```bash
ros2 launch generic_motor_driver complete_robot_navigation.launch.py map:=/path/to/your/map.yaml
```

This launches:
- Your robot (motor driver, lidar, etc.)
- Map server (loads your saved map)
- Nav2 navigation stack
- RViz

## Configuration

Nav2 is configured in `config/nav2_params.yaml` with settings optimized for your robot:

- **Robot radius**: 0.4m (based on your 0.57m wheelbase)
- **Max velocity**: 0.5 m/s linear, 1.0 rad/s angular
- **Base frame**: `base_footprint`
- **Laser scan**: `/scan` topic
- **Map frame**: `map`
- **Odometry frame**: `odom`

## Usage

### Step 1: Create a Map (if you don't have one)
```bash
# Launch mapping
ros2 launch generic_motor_driver complete_robot_mapping.launch.py

# Drive around to map the environment
# Save the map when done:
ros2 run nav2_map_server map_saver_cli -f ~/test_ws/map/my_map
```

### Step 2: Navigate with Saved Map
```bash
# Launch navigation with your map
ros2 launch generic_motor_driver complete_robot_navigation.launch.py \
  map:=~/test_ws/map/my_map.yaml
```

### Step 3: Set Initial Pose
In RViz:
1. Click "2D Pose Estimate" tool
2. Click and drag on the map where your robot is located
3. Drag to set the robot's orientation

### Step 4: Send Navigation Goal
In RViz:
1. Click "2D Nav Goal" tool
2. Click on the map where you want the robot to go
3. Drag to set the desired orientation
4. The robot will autonomously navigate to that location!

## Nav2 Features

- **Path Planning**: Plans optimal paths around obstacles
- **Obstacle Avoidance**: Uses lidar to avoid obstacles in real-time
- **Recovery Behaviors**: Spins, backs up, or drives on heading if stuck
- **Costmaps**: Local and global costmaps for safe navigation
- **AMCL**: Localization using the map and lidar scans

## Tuning Nav2

Key parameters in `config/nav2_params.yaml`:

### Robot Size
- `robot_radius`: 0.4m (adjust if robot is larger/smaller)
- `inflation_radius`: 0.9m (safety margin around obstacles)

### Speed Limits
- `max_vel_x`: 0.5 m/s (max forward speed)
- `max_vel_theta`: 1.0 rad/s (max rotation speed)
- `acc_lim_x`: 2.5 m/s² (acceleration limit)

### Goal Tolerance
- `xy_goal_tolerance`: 0.25m (how close to goal)
- `yaw_goal_tolerance`: 0.25 rad (~14 degrees)

## Troubleshooting

### Robot not moving
- Check Nav2 lifecycle: `ros2 lifecycle list /navigator`
- Check for errors: `ros2 topic echo /navigator/feedback`
- Verify cmd_vel is being published: `ros2 topic echo /cmd_vel`

### Robot gets stuck
- Increase `inflation_radius` for more clearance
- Decrease `robot_radius` if robot is smaller
- Check for obstacles in costmap visualization in RViz

### Poor localization
- Ensure initial pose is set correctly
- Check that map matches current environment
- Verify lidar is working: `ros2 topic echo /scan`

## Advanced: Mapping + Navigation Together

You can run both SLAM and Nav2 together for simultaneous localization and mapping (SLAM):

```bash
ros2 launch generic_motor_driver complete_robot_mapping.launch.py use_nav2:=true
```

This allows the robot to:
- Build a map while navigating
- Use the partial map for navigation
- Continuously improve the map

## Next Steps

1. **Test basic navigation**: Create a simple map and test navigation
2. **Tune parameters**: Adjust speeds and tolerances for your needs
3. **Add waypoints**: Use waypoint follower for multi-point navigation
4. **Explore autonomously**: Add an explorer node for autonomous mapping





