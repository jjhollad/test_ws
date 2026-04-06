# SLAM Tuning Guide

## Understanding Map Drift

Map drift occurs when the SLAM algorithm loses track of where the robot is relative to the map. This can happen due to:
1. **Poor odometry** - wheel slippage, incorrect wheel_base, or encoder issues
2. **SLAM parameters too loose** - not enough scan matching constraints
3. **SLAM parameters too tight** - rejecting valid loop closures
4. **Scan quality issues** - noisy lidar data, insufficient features

## Diagnostic Commands

### Check Odometry Quality
```bash
# Monitor odometry topic
ros2 topic echo /odom

# Check TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Visualize odometry vs map
# In RViz, add both /odom and /map displays
```

### Check Scan Quality
```bash
# View scan data
ros2 topic echo /scan

# Check scan frequency
ros2 topic hz /scan
```

### Check SLAM Status
```bash
# View SLAM Toolbox logs
ros2 topic echo /slam_toolbox/feedback

# Check map update rate
ros2 topic hz /map
```

## Key Parameters to Tune

### 1. Odometry Frame Settings
**Location**: `mapper_params_online_async.yaml`
- `odom_frame: odom` - Must match your odometry frame
- `base_frame: base_footprint` - Must match your base frame
- `scan_topic: scan` - Must match your lidar topic

### 2. Map Update Frequency
**Parameters**:
- `minimum_travel_distance: 0.25` - Update map after moving 0.25m
- `minimum_travel_heading: 0.5` - Update map after rotating 0.5 rad (~29°)
- `map_update_interval: 1.0` - Maximum time between updates (seconds)

**Tuning**:
- **Too high** (e.g., 0.5m, 1.0 rad): Map updates too infrequently, may miss details
- **Too low** (e.g., 0.1m, 0.1 rad): Map updates too often, may cause drift
- **Recommended**: Start with 0.25m, 0.5 rad, adjust based on robot speed

### 3. Scan Matching Quality
**Parameters**:
- `link_match_minimum_response_fine: 0.1` - Minimum match quality (0-1)
- `link_scan_maximum_distance: 1.5` - Max distance to search for matches (meters)
- `correlation_search_space_dimension: 0.5` - Search radius for scan matching (meters)
- `correlation_search_space_resolution: 0.01` - Search resolution (meters)

**Tuning**:
- **Increase `link_match_minimum_response_fine`** (e.g., 0.2-0.3): Require better matches, reduces false matches but may reject valid ones
- **Decrease `link_scan_maximum_distance`** (e.g., 1.0): Search closer, faster but may miss matches
- **Increase `correlation_search_space_dimension`** (e.g., 0.75): Search wider, better for fast movement but slower

### 4. Loop Closure
**Parameters**:
- `do_loop_closing: true` - Enable/disable loop closure
- `loop_search_maximum_distance: 3.0` - Max distance to search for loops (meters)
- `loop_match_minimum_chain_size: 10` - Minimum scans to form a loop
- `loop_match_minimum_response_coarse: 0.35` - Coarse match threshold
- `loop_match_minimum_response_fine: 0.45` - Fine match threshold

**Tuning**:
- **Increase thresholds** (0.4-0.5): Require better loop matches, reduces false loops
- **Decrease `loop_search_maximum_distance`** (2.0): Search closer, faster but may miss loops
- **Increase `loop_match_minimum_chain_size`** (15-20): Require more consistent matches

### 5. Scan Buffer
**Parameters**:
- `scan_buffer_size: 10` - Number of scans to keep in buffer
- `scan_buffer_maximum_scan_distance: 10.0` - Max distance between scans in buffer

**Tuning**:
- **Increase `scan_buffer_size`** (15-20): Keep more history, better for loop closure but uses more memory
- **Decrease `scan_buffer_maximum_scan_distance`** (8.0): Only keep nearby scans, faster

### 6. Variance Penalties
**Parameters**:
- `distance_variance_penalty: 0.5` - Penalty for distance uncertainty
- `angle_variance_penalty: 1.0` - Penalty for angle uncertainty

**Tuning**:
- **Increase penalties** (0.7, 1.5): Trust odometry less, rely more on scan matching
- **Decrease penalties** (0.3, 0.7): Trust odometry more, use scan matching less

## Common Issues and Solutions

### Issue: Map Drifts Continuously
**Symptoms**: Map keeps shifting, never aligns properly
**Solutions**:
1. Check odometry quality - verify wheel_base, wheel_radius are correct
2. Increase `link_match_minimum_response_fine` to 0.2-0.3
3. Decrease `correlation_search_space_dimension` to 0.3-0.4
4. Increase `distance_variance_penalty` and `angle_variance_penalty`

### Issue: Map Doesn't Update
**Symptoms**: Map stays static, doesn't grow
**Solutions**:
1. Decrease `minimum_travel_distance` to 0.1-0.15
2. Decrease `minimum_travel_heading` to 0.2-0.3
3. Check scan topic is publishing: `ros2 topic echo /scan`

### Issue: False Loop Closures
**Symptoms**: Map jumps when robot hasn't looped
**Solutions**:
1. Increase `loop_match_minimum_response_coarse` to 0.4-0.5
2. Increase `loop_match_minimum_response_fine` to 0.5-0.6
3. Increase `loop_match_minimum_chain_size` to 15-20

### Issue: Map Too Noisy
**Symptoms**: Map has lots of artifacts, ghost walls
**Solutions**:
1. Increase `link_match_minimum_response_fine` to 0.2-0.3
2. Decrease `max_laser_range` if lidar sees too far
3. Check lidar quality - clean sensor, check for reflections

## Recommended Starting Values for Your Robot

Based on your robot (0.57m wheelbase, 0.12m wheel radius):

```yaml
minimum_travel_distance: 0.2        # Update more frequently
minimum_travel_heading: 0.4         # Update after smaller rotations
link_match_minimum_response_fine: 0.15  # Slightly stricter matching
correlation_search_space_dimension: 0.4   # Smaller search area
distance_variance_penalty: 0.6      # Trust odometry less
angle_variance_penalty: 1.2         # Trust rotation less
loop_match_minimum_response_coarse: 0.4  # Stricter loop closure
loop_match_minimum_response_fine: 0.5
```

## Testing Procedure

1. **Start with default parameters** - establish baseline
2. **Drive in a small loop** (5-10m diameter) - check if map closes
3. **Drive in a larger loop** - check for drift
4. **Adjust one parameter at a time** - test after each change
5. **Document what works** - keep notes on parameter values

## Advanced: Odometry Tuning

If map drift persists, the issue may be odometry, not SLAM:

1. **Verify wheel_base**: Measure actual distance between wheel centers
2. **Verify wheel_radius**: Measure actual wheel radius
3. **Check encoder_reduction_factor**: May need adjustment
4. **Check for wheel slippage**: Drive on different surfaces, compare

Use `ros2 topic echo /odom` to monitor odometry quality.






