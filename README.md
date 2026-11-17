# ELAF Nav2 Mission (ROS 2 Jazzy)

This package launches the F1TENTH Gazebo simulation, Nav2, an Ackermann
steering bridge, RViz, and an automatic four-waypoint mission for three laps.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- F1TENTH Gym ROS (`f1tenth_gym_ros`)
- Nav2 packages for Jazzy
- Python packages: `numpy` and `yaml`

## Build

Place this package in the `src` directory of a ROS 2 workspace. For example,
link it into `~/ros2_ws/src`:

```bash
mkdir -p ~/ros2_ws/src
ln -s "$PWD/jazzy_version/elaf_nav2_mission_jazzy" \
  ~/ros2_ws/src/elaf_nav2_mission_jazzy
cd ~/ros2_ws
colcon build --packages-select elaf_nav2_mission_jazzy
source install/setup.bash
```

If `f1tenth_gym_ros` is not already built in the same workspace, build it
before the mission package.

## Launch

From a workspace with the package built and sourced:

```bash
ros2 launch elaf_nav2_mission_jazzy elaf_nav2.launch.py
```

This launch starts:

- the existing F1TENTH Gym ROS simulator;
- MPPI Ackermann control and the Hybrid A* planner;
- Nav2 costmaps, behaviors, lifecycle management, and the behavior tree
  navigator;
- the Ackermann bridge;
- the automatic three-lap mission; and
- RViz.

## Launch arguments

Override the launch arguments after the launch file name:

```bash
ros2 launch elaf_nav2_mission_jazzy elaf_nav2.launch.py \
  max_speed:=4.0 autostart_mission:=true rviz:=true foxglove_port:=8765
```

| Argument | Default | Description |
|---|---:|---|
| `max_speed` | `""` | Maximum forward speed. When empty, the package uses `6.0 m/s` from `nav2_params.yaml`. |
| `autostart_mission` | `true` | Start the automatic lap mission. Set to `false` to launch Nav2 and RViz only. |
| `rviz` | `true` | Open RViz. Set to `false` for a headless launch. |
| `foxglove_port` | `8765` | Port passed to the F1TENTH Gym ROS bridge. |

## Mission configuration

Edit the mission parameters in:

```text
jazzy_version/elaf_nav2_mission_jazzy/config/nav2_params.yaml
```

The default mission uses four waypoints and three laps:

```yaml
mission:
  ros__parameters:
    laps: 3
    waypoint_1: [10.15, 3.0, 1.5707963268]
    waypoint_2: [-5.5, 8.15, 3.1415926536]
    waypoint_3: [-13.7, 6.0, -1.5707963268]
    waypoint_4: [-11.5, 0.0, 0.0]
```

Waypoint values are `[x, y, yaw]` in meters and radians in the `map` frame.
After editing `nav2_params.yaml`, rebuild the package so the updated file is
installed:

```bash
cd ~/ros2_ws
colcon build --packages-select elaf_nav2_mission_jazzy
source install/setup.bash
```

## Troubleshooting

If the package is not found:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix elaf_nav2_mission_jazzy
```

If the launch reports a simulator configuration error, verify that
`f1tenth_gym_ros/config/sim.yaml` uses the required Levine map profile:

- `levine_obs` map
- `f1tenth` vehicle parameters
- scale `1.0`
- wall-clock time (`use_sim_time: false`)
