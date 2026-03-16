# DAL Webots Simulator

A multi-robot simulation framework built on [Webots R2025a](https://cyberbotics.com/). Supports autonomous waypoint navigation, LIDAR-based occupancy grid mapping, and real-time visualization for KUKA YouBot and Pioneer 3-AT robots.

Designed as a standardized base for robotics research studies. Study-specific code (planners, controllers, worlds) lives in `study/*` branches.

---

## Quick Start

**Requirements:** Webots R2025a, Python 3.7+

```bash
pip install -r requirements.txt
```

1. Open `worlds/DAL.wbt` in Webots and press Play
2. `python tools/visualizer.py`
3. Drive with keyboard (controller: `dal_controller`) or run `python planners/simple_planner.py` (controller: `waypoint_controller`)

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
│   ├── visualizer.py           2D occupancy grid + robot positions
│   ├── camera_viewer.py        Live camera feed window
│   └── grid_visualization.py   Simple position-only grid overlay
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

| Branch | Description | Extra deps |
|--------|-------------|------------|
| `study/ltl-exploration` | LTL temporal logic multi-robot exploration | `spot`, `buddy`, `pupil-apriltags` (Linux) |

To start a new study, see [docs/branching_for_study.md](docs/branching_for_study.md).

---

## Known Issues

- Controllers still import `from dal.protocol` (old folder name). Until fixed, create a symlink: `ln -s utils dal` from the project root. See [docs/getting_started.md](docs/getting_started.md).
