# Greenship Pathfinder

> **Project Status:** Archived / Legacy. This repository contains the autonomous navigation and path-planning codebase developed for Autonomous Surface Vehicle (ASV) operations. Active development has concluded; the codebase is preserved for technical reference and portfolio review.

---

## 1. Overview

`greenship_pathfinder` is a ROS 1 (Catkin) package designed for autonomous surface vessel guidance, obstacle avoidance, and path planning. The system interfaces with flight controllers (ArduPilot / PX4) via MAVROS and MAVLink, maps environmental obstacles into a local North-East-Down (NED) coordinate frame, and computes collision-free trajectories using potential-field-augmented A* search.

### Core Capabilities
- **Coordinate Transformation:** Converts WGS-84 global coordinates (latitude/longitude) to a local metric NED grid centered on the vehicle home position.
- **Path Planning:** Executes A* search on a 2D occupancy grid with exponential potential-field penalty layers for static obstacles, dynamic obstacles, and navigation buoys.
- **Buoy Channel Navigation:** Matches red (port) and green (starboard) buoys using KD-Tree nearest-neighbor queries and generates center-corridor paths.
- **Vehicle Control Interface:** Manages vehicle state transitions (`MANUAL`, `GUIDED`, `AUTO`), arming/disarming sequences, and global waypoint transmission via MAVROS service and topic interfaces.
- **Sensor Integration & Fusion:** Ingests detections from YOLO-based vision models and LiDAR range sensors to localize dynamic and static marine obstacles.

---

## 2. System Architecture

```
                      +-----------------------------+
                      |   Sensors / Perception      |
                      |  - Camera (YOLO detection)  |
                      |  - LiDAR (LaserScan)        |
                      +--------------+--------------+
                                     |
                                     v
                      +-----------------------------+
                      |    Sensor Fusion Node       |
                      |  (/fusion_output/* topics)  |
                      +--------------+--------------+
                                     |
                                     v
+-------------------------------------------------------------------------+
| autopath_core Node                                                      |
|                                                                         |
|  +------------------------+             +----------------------------+  |
|  |     Commander          |             |       NavDomain            |  |
|  |  - WGS84 <-> NED Frame |<----------->|  - 2D Occupancy Grid       |  |
|  |  - KD-Tree Buoy Pairs  |             |  - Potential Fields Cost   |  |
|  |  - Waypoint Tracking   |             |  - A* Path Search          |  |
|  +-----------+------------+             +----------------------------+  |
|              |                                                          |
|              v                                                          |
|  +------------------------+                                             |
|  |   Vehicle Interface    |                                             |
|  |  - Mode & Arm Services |                                             |
|  |  - Setpoint Publisher  |                                             |
|  +-----------+------------+                                             |
+--------------|----------------------------------------------------------+
               |
               v
+-------------------------------------------------------------------------+
| MAVROS / Autopilot (PX4 / ArduPilot)                                    |
| - Topics: /mavros/state, /mavros/global_position/global, etc.           |
| - Services: /mavros/cmd/arming, /mavros/set_mode                        |
+-------------------------------------------------------------------------+
```

---

## 3. Package Structure

```
greenship_pathfinder/
├── CMakeLists.txt              # Catkin build configuration and script installation
├── package.xml                 # ROS package manifest and dependency definitions
├── setup.py                    # Python package setup for autopath_core
│
├── src/
│   └── autopath_core/          # Core library
│       ├── __init__.py
│       ├── core.py             # Commander class, mission state machine, frame transforms
│       ├── pathfinder.py       # NavDomain class, cost grid, A* search implementation
│       └── vehicle_interface.py# Vehicle class, MAVROS topic/service communication
│
├── scripts/                    # Executable ROS nodes
│   ├── autopath_core_starter.py       # Main entry point for autonomous mission execution
│   ├── collision_avoidance.py         # Collision avoidance mission controller
│   ├── maneuvering.py                 # Buoy channel navigation controller
│   ├── collision_avoidance_simulator.py # Simulation node with moving obstacles and gates
│   └── dynamic_obstacle_simulator.py  # Standalone dynamic obstacle generator
│
├── launch/                     # ROS launch files
│   ├── autopath_core.launch                 # Launch autopath_core node
│   ├── autopath_core_debug.launch           # Launch autopath_core with DEBUG logging
│   ├── collision_avoidance_task.launch      # Launch collision avoidance task node
│   ├── collision_avoidance_environment.launch # Launch collision simulation environment
│   └── dynamic_obstacle.launch              # Launch obstacle simulator
│
├── njord_testing/              # Full-system test launch files and sensor scripts
│   ├── navigation.launch       # Multi-mode navigation launcher (MAVROS + controllers)
│   ├── sensors_fusion.launch   # Sensor pipeline launcher (LiDAR + Camera + Fusion)
│   ├── fusion_v3.py            # LiDAR-camera data fusion implementation
│   ├── cv_kod_v4.py            # YOLO camera detection node
│   └── lidar_yayin_v3.py       # LiDAR range preprocessor
│
└── path_finding.py             # Standalone MAVLink/YOLO navigation script (Jetson/Rover)
```

---

## 4. Component Details

### `autopath_core.pathfinder.NavDomain`
- Maintains a discrete grid (`self.map`) of configurable dimensions.
- Calculates exponential potential-field decay around obstacles and buoys:
  $$\text{value}(d) = \exp\left(\ln(V) \cdot \frac{R - d}{R - 1}\right)$$
  where $V$ is the base obstacle weight, $R$ is influence radius, and $d$ is Euclidean distance.
- Implements A* search with cost function:
  $$f(n) = g(n) + h(n) - \text{attractval}(n) + \text{repelval}(n) + \text{penalty}_{\text{corridor}} + \text{penalty}_{\text{moving\_obstacle}}$$

### `autopath_core.vehicle_interface.Vehicle`
- Wraps MAVROS communication using thread-safe state variables.
- Subscribes to `/mavros/state`, `/mavros/home_position/home`, `/mavros/global_position/global`, `/mavros/vfr_hud`, and `/mavros/mission/waypoints`.
- Invokes `/mavros/cmd/arming` and `/mavros/set_mode` services.
- Publishes target setpoints to `/mavros/setpoint_position/global` (`geographic_msgs/GeoPoseStamped`).

### `autopath_core.core.Commander`
- Translates between latitude/longitude (WGS-84) and metric NED coordinates using spherical Earth projection ($R = 6{,}378{,}137\text{ m}$).
- Uses `scipy.spatial.KDTree` to index detected buoys and pair red/green markers to construct navigation corridors.
- Monitors data freshness timeouts for obstacle streams and vehicle telemetry.
- Executes the waypoint tracking loop with dynamic look-ahead waypoint selection.

---

## 5. ROS Interface

### Subscribed Topics
| Topic | Message Type | Description |
|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | Connection, arming, and flight mode state |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | Vehicle GPS position |
| `/mavros/home_position/home` | `mavros_msgs/HomePosition` | Reference home position |
| `/mavros/vfr_hud` | `mavros_msgs/VFR_HUD` | Compass heading and speed |
| `/mavros/mission/waypoints` | `mavros_msgs/WaypointList` | Mission waypoint list |
| `/fusion_output/dynamic_obstacles` | `std_msgs/Float32MultiArray` | Polar coordinates $[r_i, \theta_i]$ of dynamic obstacles |
| `/fusion_output/static_obstacles` | `std_msgs/Float32MultiArray` | Polar coordinates $[r_i, \theta_i]$ of static obstacles |
| `/fusion_output/red_buoy` | `std_msgs/Float32MultiArray` | Polar coordinates $[r_i, \theta_i]$ of port buoys |
| `/fusion_output/green_buoy` | `std_msgs/Float32MultiArray` | Polar coordinates $[r_i, \theta_i]$ of starboard buoys |

### Published Topics
| Topic | Message Type | Description |
|---|---|---|
| `/mavros/setpoint_position/global` | `geographic_msgs/GeoPoseStamped` | Global target coordinate dispatched to autopilot |
| `autopath_core/vehicle_relative_pose` | `geometry_msgs/PoseStamped` | Vehicle pose in local NED coordinate frame |

### Service Clients
| Service | Service Type | Description |
|---|---|---|
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | Vehicle arm / disarm command |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | Autopilot mode change (`GUIDED`, `MANUAL`, `AUTO`) |

---

## 6. Dependencies & Requirements

### System Requirements
- Ubuntu 20.04 LTS
- ROS Noetic (or ROS Melodic on Ubuntu 18.04)
- Python 3.8+

### ROS Dependencies
- `catkin`
- `rospy`
- `mavros`
- `mavros_msgs`
- `geographic_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `std_msgs`

### Python Dependencies
- `numpy`
- `scipy`
- `opencv-python`
- `torch` / `ultralytics` (for vision detection scripts)
- `pymavlink` (for standalone telemetry scripts)
- `matplotlib` (for simulation visualization)

---

## 7. Build and Execution

### Build Workspace
```bash
# Source ROS environment
source /opt/ros/noetic/setup.bash

# Navigate to catkin workspace
cd ~/catkin_ws/src

# Clone repository
git clone https://github.com/ismail-ciftci/greenship_pathfinder.git

# Build package
cd ~/catkin_ws
catkin_make --pkg greenship_pathfinder

# Source workspace
source devel/setup.bash
```

### Launch Autonomous Navigation
```bash
# Launch main autopath core node
roslaunch greenship_pathfinder autopath_core.launch

# Launch collision avoidance task node
roslaunch greenship_pathfinder collision_avoidance_task.launch
```

### Launch Simulation Environment
```bash
# Run collision avoidance simulation with moving obstacles
roslaunch greenship_pathfinder collision_avoidance_environment.launch
```

---
