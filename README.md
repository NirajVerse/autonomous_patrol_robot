# Autonomous Patrol Robot

A ROS 2 and Gazebo Sim project for an autonomous TurtleBot3 inspection robot. The robot follows a predefined road network, visits three patrol stations, detects blocked roads with LiDAR, returns to the previous intersection, and replans around the obstruction.

This repository is an educational prototype. It uses a lightweight custom controller and graph-based route planner instead of the ROS 2 Navigation Stack (Nav2), which makes the navigation and recovery logic easier to inspect and modify.

## Project overview

The simulated environment contains a small road network with intersections `A` through `F` and three inspection stations:

- `P1` at the upper-left end of the network
- `P2` at the lower-left end of the network
- `P3` at the upper-right end of the network

The robot starts near `(0.0, -1.3)` and attempts to visit `P1`, `P2`, and `P3` in that order.

```text
Gazebo patrol world
        |
        v
  /odom and /scan
        |
        v
  Patrol manager <---- Graph route planner
        |
        v
     /cmd_vel
        |
        v
    TurtleBot3
```

The main patrol manager uses the following state machine:

```text
PLAN -> ROTATE -> DRIVE
                    |
                    | obstacle detected
                    v
          RECOVER_ROTATE -> RECOVER_DRIVE -> PLAN
```

When an obstacle is detected, the current graph edge is marked as blocked. The robot turns around, returns to the previous intersection, and asks the route planner for a new path that excludes that edge.

## Requirements

The current workspace was developed for:

- Ubuntu with ROS 2 Jazzy
- Gazebo Sim through the ROS-Gazebo integration packages
- TurtleBot3 simulation packages
- Python 3
- `colcon` and `rosdep`

The launch files depend on `turtlebot3_gazebo`, `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`, and `robot_state_publisher`.

The examples below assume ROS 2 Jazzy is installed under `/opt/ros/jazzy`.

## Clone and build

Clone the repository with its TurtleBot3 simulation submodule:

```bash
git clone --recurse-submodules <repository-url>
cd inspection_robot_ws
```

If the repository was already cloned without submodules, initialize them with:

```bash
git submodule update --init --recursive
```

Install package dependencies and build the workspace:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Run the `source` commands again in every new terminal. They may also be added to the shell configuration if desired.

## Run the patrol

Open two terminals in the workspace root.

### Terminal 1: start the simulation

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch "$(pwd)/launch/patrol.launch.py"
```

This starts Gazebo Sim, loads `worlds/patrol.world`, spawns the TurtleBot3 Burger near the bottom of the road network, and starts the required ROS-Gazebo bridges.

Wait until the robot and world are fully visible before starting the patrol controller.

### Terminal 2: start the patrol manager

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/patrol_manager.py
```

The controller waits for both odometry and laser scan data. It then calculates a route, aligns the robot with the first road segment, and begins the patrol.

Press `Ctrl+C` in the controller terminal to stop the robot. Stop the Gazebo launch separately in the first terminal.

## Expected behavior

During a successful run, the controller should:

1. Calculate a shortest route from `START` to `P1`.
2. Rotate toward and follow each road segment.
3. Record `P1` as visited using its odometry estimate.
4. Repeat the process for `P2` and `P3`.
5. Print a final patrol report.

If LiDAR detects an obstacle closer than the configured threshold while driving, the controller should:

1. Stop the robot.
2. Mark the current road as blocked.
3. Turn around when sufficient clearance is available.
4. Return to the previous graph node.
5. Replan without using the blocked road.

Useful ROS topics include:

| Topic | Message type | Purpose |
| --- | --- | --- |
| `/odom` | `nav_msgs/msg/Odometry` | Robot pose estimate |
| `/scan` | `sensor_msgs/msg/LaserScan` | Obstacle measurements |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Robot velocity commands |

## Repository layout

```text
inspection_robot_ws/
|-- create_patrol_map.py       # Generates the patrol world and base launch file
|-- launch/
|   |-- patrol.launch.py       # Starts the world, robot, bridges, and state publisher
|   `-- spawn_patrol_robot.launch.py
|-- models/
|   `-- obstacle.sdf           # Reusable obstacle model
|-- scripts/
|   |-- patrol_manager.py      # Current integrated patrol controller
|   |-- route_planner.py       # Weighted graph and shortest-path search
|   |-- detect_obstacle.py     # Basic LiDAR experiment
|   |-- follow_waypoints.py    # Waypoint-following experiment
|   |-- move_to_intersection.py
|   |-- road_controller.py     # Road center-line controller
|   |-- test_turn.py
|   |-- test_two_roads.py
|   |-- test_recovery.py
|   `-- test_rerouting.py
|-- src/
|   `-- turtlebot3_simulations # Upstream simulation dependency (Git submodule)
`-- worlds/
    `-- patrol.world           # Generated custom Gazebo world
```

The `build/`, `install/`, and `log/` directories are generated by ROS tooling and are intentionally excluded from Git.

## Run individual experiments

The scripts under `scripts/` document the incremental development of the controller. Start the simulation first, then run only one motion-control script at a time. Multiple nodes publishing to `/cmd_vel` will compete for control of the robot.

Examples:

```bash
# Print the planned station routes without starting ROS
python3 scripts/route_planner.py

# Test graph rerouting around selected blocked roads
python3 scripts/test_rerouting.py

# Observe basic LiDAR obstacle distances in a running simulation
python3 scripts/detect_obstacle.py

# Run a simple rotation experiment
python3 scripts/test_turn.py
```

Other motion experiments can be started in the same way:

```bash
python3 scripts/move_to_intersection.py
python3 scripts/follow_waypoints.py
python3 scripts/road_controller.py
python3 scripts/test_two_roads.py
python3 scripts/test_recovery.py
```

## Regenerate the patrol world

`create_patrol_map.py` reads the upstream TurtleBot3 empty world, adds roads and patrol stations, and regenerates `worlds/patrol.world` and `launch/patrol.launch.py`.

```bash
python3 create_patrol_map.py
```

Review the resulting changes before committing them because this command overwrites both generated files.

## Current limitations

- Station visits are inferred from odometry; there is no physical or visual station detector.
- Navigation depends on a known, fixed road graph and an accurate initial pose.
- The controller does not use SLAM, Nav2, a costmap, or global localization.
- Collision checks use simple LiDAR distance thresholds rather than the full robot footprint.
- The obstacle model is provided separately and is not automatically inserted into the patrol world.
- Motion speeds and control gains are hard-coded for the current simulation.
- The custom Python controllers are workspace scripts rather than an installable ROS 2 package.
- The experiment scripts are manual tests, not an automated integration test suite.

## Troubleshooting

### `KeyError: 'TURTLEBOT3_MODEL'`

Set the robot model before launching:

```bash
export TURTLEBOT3_MODEL=burger
```

### Package not found

Make sure both ROS and the workspace overlay are sourced:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

If the workspace has not been built yet, run `colcon build --symlink-install` first.

### No `/scan` or `/odom` data

Confirm that the simulation and bridge nodes are running:

```bash
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /scan --once
```

### Robot does not move

Check that only one controller is publishing velocity commands:

```bash
ros2 topic info /cmd_vel --verbose
```

Also inspect the patrol-manager terminal for messages about missing sensor data, excessive lateral deviation, an obstacle, or insufficient turning clearance.

### Reset after a failed experiment

Stop the controller and simulation, restart the launch file, wait for the robot to spawn fully, and then start the desired controller again. The patrol manager stores blocked roads only in memory, so restarting it clears the current patrol state.

## Safety note

This repository currently targets simulation. The hard-coded speeds, clearances, coordinate transforms, and recovery behavior should not be used on a physical robot without additional validation, emergency-stop handling, and collision-safety testing.
