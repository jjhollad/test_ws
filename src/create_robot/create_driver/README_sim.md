# Classic Gazebo + Nav2 Simulation Quickstart

This guide shows how to run a repeatable Nav2 tuning workflow in simulation using Gazebo Classic (`gzserver` / `gzclient`) on ROS 2 Humble.

Key simulation files in this package:

- `launch/classic_nav2_sim.launch.py`
- `launch/rectangular_classic_nav2_sim.launch.py`
- `config/nav2_params_sim.yaml`
- `worlds/turtlebot3_world_spacious.world`
- `worlds/square_building_10ft_hallway.world`

These are intended for fast iteration on path planning and controller tuning before applying changes to the real robot.

---

## 1) One-time prerequisites

Install required simulation packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-msgs
```

Build your workspace:

```bash
cd /home/user/test_ws
colcon build --packages-select generic_motor_driver
```

---

## 2) Start simulation

In a fresh terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/user/test_ws/install/setup.bash
ros2 launch generic_motor_driver classic_nav2_sim.launch.py
```

Default behavior:

- Gazebo server + client start
- RViz starts
- Nav2 runs with `config/nav2_params_sim.yaml`
- `use_sim_time` is enabled

---

## 3) Launch options and world selection

Headless (no Gazebo GUI):

```bash
ros2 launch generic_motor_driver classic_nav2_sim.launch.py headless:=True use_rviz:=False
```

SLAM mode (instead of static map localization):

```bash
ros2 launch generic_motor_driver classic_nav2_sim.launch.py slam:=True
```

Use custom params file:

```bash
ros2 launch generic_motor_driver classic_nav2_sim.launch.py \
  params_file:=/home/user/test_ws/src/create_robot/create_driver/config/nav2_params_sim.yaml
```

Run your rectangular robot model (URDF + Gazebo diff drive plugin):

```bash
ros2 launch generic_motor_driver rectangular_classic_nav2_sim.launch.py
```

Rectangular launch defaults:

- world: `worlds/turtlebot3_world_spacious.world`
- spawn: hallway-ready default (`spawn_x`, `spawn_y`, `spawn_z`)

Optional flags:

```bash
ros2 launch generic_motor_driver rectangular_classic_nav2_sim.launch.py headless:=True
ros2 launch generic_motor_driver rectangular_classic_nav2_sim.launch.py slam:=False
```

Use the custom square-building world (10 ft hallway with wall-attached jut-outs):

```bash
ros2 launch generic_motor_driver rectangular_classic_nav2_sim.launch.py \
  world:=/home/user/test_ws/src/create_robot/create_driver/worlds/square_building_10ft_hallway.world
```

Override spawn point (useful if robot starts in the wrong area):

```bash
ros2 launch generic_motor_driver rectangular_classic_nav2_sim.launch.py \
  spawn_x:=11.9 spawn_y:=0.0 spawn_z:=0.01
```

---

## 4) Basic operation in RViz

1. Click **2D Pose Estimate** to initialize localization.
2. Click **Nav2 Goal** to send a target pose.
3. Observe:
   - path smoothness
   - cornering behavior
   - goal approach and final heading
   - obstacle clearance

If behavior is unstable, restart and retest the same goal to keep comparisons fair.

---

## 5) Recommended tuning loop

Change only a few parameters at once, then retest with the same 2-3 goals.

Suggested order:

1. **Controller finish behavior**
   - `controller_server.FollowPath.RotateToGoal.*`
   - `controller_server.general_goal_checker.*`
2. **Path tracking / cornering**
   - `PathAlign.*`, `GoalDist.scale`, `PathDist.scale`
3. **Speed limits / smoothness**
   - `max_speed_xy`, `max_vel_theta`, `velocity_smoother.*`
4. **Obstacle clearance / shape**
   - `footprint` polygon and `inflation_radius`

After each change:

```bash
cd /home/user/test_ws
colcon build --packages-select generic_motor_driver
source install/setup.bash
ros2 launch generic_motor_driver rectangular_classic_nav2_sim.launch.py
```

---

## 6) Debug checks

Check command stream:

```bash
ros2 topic echo /cmd_vel
```

Check odometry:

```bash
ros2 topic echo /odom
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

If nav appears to fight manual control, ensure only one teleop/nav stack is running.

Check who is publishing commands:

```bash
ros2 topic info -v /cmd_vel
```

If stuck with stale processes:

```bash
./src/create_robot/create_driver/scripts/kill_robot_stack.sh
```

---

## 7) Applying sim results to hardware

Keep simulation tuning in `nav2_params_sim.yaml`.
Once stable, copy only validated settings into `config/nav2_params.yaml` for the real robot and test at low speed first.

