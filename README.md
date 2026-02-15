# DAL Simulator Package

---

## Repository Structure

```
DAL/
├── worlds/
│   └── DAL.wbt                # Webots world (factory, robots, sensors)
├── controllers/               # Run INSIDE Webots - control robots
│   ├── dal_controller/        # Teleop + LIDAR/camera/position streaming
│   ├── youbot_dal/            # Teleop + position-only streaming
│   ├── waypoint_sample/       # Drive to hardcoded waypoints
│   └── waypoint_controller/   # Receives waypoints from planner (TCP)
├── planners/                  # Run on HOST - send waypoints to controllers
│   └── simple_planner.py      # Sends 4 waypoints one-at-a-time
├── tools/                     # Run on HOST - monitoring/visualization
│   ├── visualizer.py          # 2D occupancy grid + robot positions (UDP)
│   ├── camera_viewer.py       # Live camera window (UDP)
│   └── grid_visualization.py  # Alternative grid display
└── dal/                       # Shared library (protocol, occupancy grid)
    ├── protocol.py            # Ports, packet formats (position, camera, waypoints)
    └── occupancy_grid.py      # Log-odds grid, LIDAR ray casting, cell freeze
```

- **controllers/**: Run inside Webots. Control robots directly. Assigned in world file.
- **planners/**: Run on host. Send waypoints to controllers via TCP.
- **tools/**: Run on host. Monitor sensors via UDP (passive, read-only).
- **dal/**: Shared library used by all components.



## Prerequisites

- [Webots R2025a](https://cyberbotics.com/) (exact version required)
- Python 3.7+

Install Python dependencies:
```bash
pip install -r requirements.txt
```

---

## Quick Start

1. Open `worlds/DAL.wbt` in Webots and start the simulation.
2. **Terminal 1** (from project root):
   ```bash
   python tools/visualizer.py
   ```
   Shows 2D occupancy grid and Youbot/Pioneer positions; uses LIDAR from Youbot.
3. **Terminal 2** (optional):
   ```bash
   python tools/camera_viewer.py
   ```
   Shows live camera feed. Press `q` in the window to quit.
4. In Webots, focus the 3D view and drive:
   - **Youbot**: Arrow keys + Q/E (rotate).
   - **Pioneer**: Numpad 8/2/4/6 (forward/back/turn).

For position-only display (no LIDAR), use `grid_visualization.py` and the `youbot_dal` controller.

---

## Ports (dal/protocol.py)

| Port   | Usage                          |
|--------|--------------------------------|
| 5555   | UDP: position + heading + LIDAR |
| 5556   | UDP: camera RGB                 |
| 6000   | TCP: waypoint commands (YouBot) |
| 6001   | TCP: waypoint commands (Pioneer)|

---

## Planner-Controller Architecture (NEW)

The **waypoint_controller** and **simple_planner** provide a separation between planning (deciding where to go) and control (driving the robot).

### How It Works

```
┌──────────────────┐   TCP    ┌─────────────────────┐
│  Planner (host)  │ :6000    │ Controller (Webots) │
│  simple_planner  ├─────────>│ waypoint_controller │
│                  │ WAYPOINT │                     │
│  Sends waypoints │<─────────┤ Navigates robot     │
│  one at a time   │ REACHED  │ Streams sensors     │
└──────────────────┘          └─────────────────────┘
```

**Controller** (runs inside Webots):
- Opens TCP server on port 6000 (YouBot) or 6001 (Pioneer)
- Waits for planner to connect and send waypoints
- Navigates robot to each waypoint
- Sends "REACHED" acknowledgment when goal is reached
- Continues streaming position/LIDAR/camera over UDP

**Planner** (runs on host):
- Connects to controller via TCP
- Has a list of waypoints to visit
- Sends waypoints ONE at a time
- Waits for "REACHED" before sending next waypoint

### Usage

**Terminal 1 - Start Webots:**
1. Open `worlds/DAL.wbt` in Webots
2. Assign `waypoint_controller` to a robot's controller field
3. Start simulation

**Terminal 2 - Run Visualizer (optional):**
```bash
python tools/visualizer.py
```

**Terminal 3 - Run Planner:**
```bash
python planners/simple_planner.py
```

The planner will send 4 hardcoded waypoints to the robot one at a time. Edit `WAYPOINTS` in [simple_planner.py](planners/simple_planner.py) to customize the waypoint list. Edit `ROBOT_ID` to control a different robot (0=YouBot, 1=Pioneer).

### Protocol

Simple text-based protocol over TCP:

**Planner → Controller:**
```
WAYPOINT <x> <y>\n
```

**Controller → Planner:**
```
REACHED <x> <y>\n
```

---
