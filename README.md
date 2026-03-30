# DAL Webots Simulator

A multi-robot simulation framework built on [Webots R2025a](https://cyberbotics.com/). Supports autonomous waypoint navigation, LIDAR-based occupancy grid mapping, and real-time visualization for KUKA YouBot and Pioneer 3-AT robots.

Designed as a standardized base for robotics research studies. Study-specific code (planners, controllers, worlds) lives in `study/*` branches.

---

## Quick Start

**Requirements:** Webots R2025a, Python 3.7+

```bash
pip install -r requirements.txt
```

1. Open `worlds/DAL-Factory.wbt` in Webots and press Play
2. `python tools/slam_viz.py dal-factory`
3. Drive with keyboard (controller: `dal_controller`) or run a planner (controller: `waypoint_controller`):

```bash
python planners/simple_planner.py              # sequential waypoints, default robot
python planners/continuous_planner.py <robot_id>  # continuous path following
```

**Robot IDs:** `0` = YouBot (mecanum), `3` = Pioneer3at\_3 (differential drive)

```bash
python planners/continuous_planner.py 0   # run on YouBot
python planners/continuous_planner.py 3   # run on Pioneer3at_3
```

See [docs/getting_started.md](docs/getting_started.md) for full setup instructions.

---

## Repository Structure

```
DAL-webots-simulator/
├── worlds/                     Webots world files (.wbt)
├── controllers/                Run INSIDE Webots — robot control scripts
│   ├── dal_controller/         Teleop + full sensor streaming
│   ├── waypoint_controller/    Autonomous nav via TCP planner
│   ├── waypoint_pioneer/       Pioneer-specific waypoint nav
│   ├── youbot_dal/             Minimal teleop + position streaming
│   └── dronecontroller/        Mavic 2 Pro drone PID controller
├── planners/                   Run on HOST — mission logic
│   └── simple_planner.py       Sequential hardcoded waypoints
├── tools/                      Run on HOST — monitoring (read-only)
│   ├── slam_viz.py             2D occupancy grid + robot positions (LIDAR SLAM)
│   ├── robot_pos_viz.py        Simple position-only grid overlay
│   └── camera_viz.py           Live camera feed window
├── utils/                      Shared library (protocol, occupancy grid)
│   ├── protocol.py             Ports, packet formats, message helpers
│   └── occupancy_grid.py       Log-odds grid, Bresenham ray casting
├── docs/                       Documentation
└── requirements.txt
```

---

## Network Ports

| Port | Transport | Content |
|------|-----------|---------|
| 5555 | UDP | Position + heading + LIDAR ranges |
| 5556 | UDP | Camera RGB frames |
| 6000 | TCP | Waypoint commands for YouBot (ROBOT_ID=0) |
| 6001 | TCP | Waypoint commands for Pioneer (ROBOT_ID=1) |

---

## Documentation

| Document | Description |
|----------|-------------|
| [docs/architecture.md](docs/architecture.md) | System design, communication diagram, packet formats |
| [docs/getting_started.md](docs/getting_started.md) | Installation, first run, troubleshooting |
| [docs/adding_a_world.md](docs/adding_a_world.md) | Creating and configuring Webots worlds |
| [docs/adding_a_robot.md](docs/adding_a_robot.md) | Adding new robot types and drivers |
| [docs/adding_a_controller.md](docs/adding_a_controller.md) | Writing controllers that run inside Webots |
| [docs/adding_a_planner.md](docs/adding_a_planner.md) | Writing host-side planners |
| [docs/adding_a_sensor.md](docs/adding_a_sensor.md) | Enabling and reading LIDAR, camera, IMU, GPS |
| [docs/using_visualizer.md](docs/using_visualizer.md) | Visualizer configuration and extension |
| [docs/branching_for_study.md](docs/branching_for_study.md) | How to create a `study/*` branch for research |

---

## Study Branches

To start a new study, see [docs/branching_for_study.md](docs/branching_for_study.md).

---