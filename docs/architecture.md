# System Architecture

## Overview

DAL Webots Simulator separates concerns into three independent layers that communicate over the network:

```
┌─────────────────────────────────────────────────────────────┐
│                        HOST MACHINE                          │
│                                                             │
│  ┌─────────────┐    TCP      ┌───────────────────────────┐  │
│  │   Planner   │ WAYPOINT x y│      Webots Simulation    │  │
│  │ (planners/) │────────────>│  ┌────────────────────┐   │  │
│  │             │ REACHED x y │  │  Controller        │   │  │
│  │             │<────────────│  │  (controllers/)    │   │  │
│  └─────────────┘             │  │  Runs inside robot │   │  │
│                              │  └────────────────────┘   │  │
│  ┌─────────────┐    UDP      │                           │  │
│  │  Visualizer │<────────────│  streams pos+lidar+camera │  │
│  │  (tools/)   │             │                           │  │
│  └─────────────┘             └───────────────────────────┘  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Three Layers

### 1. Controller (runs inside Webots)

Located in `controllers/<name>/`. Each robot in the world has exactly one controller assigned to it.

**Responsibilities:**
- Read sensors (LIDAR, camera, position from Supervisor API)
- Drive actuators (wheels)
- Stream sensor data over UDP to any host-side tools
- Accept waypoint commands over TCP from a planner
- Navigate to waypoints and send `REACHED` acknowledgment

**Does not:**
- Make high-level decisions about where to go
- Do any planning or path selection

### 2. Planner (runs on host)

Located in `planners/`. Run as a regular Python script before or during simulation.

**Responsibilities:**
- Decide which waypoints to visit and in what order
- Connect to controller via TCP
- Send `WAYPOINT x y` commands one at a time
- Wait for `REACHED x y` before sending the next waypoint

**Does not:**
- Directly touch any Webots API
- Control robot actuators

### 3. Tools (runs on host)

Located in `tools/`. Run independently at any time — passive listeners.

**Responsibilities:**
- Receive UDP packets from controllers
- Display occupancy grid, robot positions, camera feed

**Does not:**
- Send any commands to the robots

---

## Network Ports

All communication is over `localhost` by default.

| Port | Transport | Direction | Data |
|------|-----------|-----------|------|
| `5555` | UDP | Controller → Tools | Robot position, heading, LIDAR ranges |
| `5556` | UDP | Controller → Tools | Camera RGB frames |
| `6000` | TCP | Planner ↔ Controller | Waypoints for Robot 0 (YouBot) |
| `6001` | TCP | Planner ↔ Controller | Waypoints for Robot 1 (Pioneer) |

Port formula: `COMMAND_PORT_BASE + ROBOT_ID` → `6000 + 0 = 6000`, `6000 + 1 = 6001`.

---

## Packet Formats

Defined in `utils/protocol.py`.

### Position + LIDAR (UDP, port 5555)

Binary, big-endian:

```
┌──────────┬──────────┬──────────┬──────────┬──────────┬──────────────────┐
│ robot_id │    x     │    y     │ heading  │  n_lidar │  lidar[0..n-1]   │
│  1 byte  │  4 bytes │  4 bytes │  4 bytes │  2 bytes │  n × 4 bytes     │
│  uint8   │  float32 │  float32 │  float32 │  uint16  │  float32 each    │
└──────────┴──────────┴──────────┴──────────┴──────────┴──────────────────┘
```

- `x`, `y`: world coordinates in meters
- `heading`: radians, 0 = +X axis, CCW positive
- `n_lidar`: number of LIDAR range readings in this packet
- `lidar[i]`: range in meters for beam `i`

### Camera (UDP, port 5556)

```
┌──────────┬──────────┬──────────┬──────────────────────────┐
│ robot_id │  width   │  height  │     RGB bytes            │
│  1 byte  │  2 bytes │  2 bytes │  width × height × 3      │
│  uint8   │  uint16  │  uint16  │  uint8 each (R,G,B)      │
└──────────┴──────────┴──────────┴──────────────────────────┘
```

### Waypoint Commands (TCP, text)

```
Planner → Controller:   WAYPOINT <x> <y>\n
Controller → Planner:   REACHED <x> <y>\n
```

Both `<x>` and `<y>` are decimal floats in meters.

---

## Robots

| Robot ID | Model | Drive | LIDAR | Controller port |
|----------|-------|-------|-------|-----------------|
| `0` | KUKA YouBot | Mecanum (omni-directional) | LDS-01, 360°, 3.5 m | 6000 |
| `1` | Pioneer 3-AT | Differential | Sick LMS 291, 180°, 8 m | 6001 |

---

## Shared Library

`utils/` contains code shared between controllers, planners, and tools.

| File | Purpose |
|------|---------|
| `utils/protocol.py` | Port constants, `pack_position`, `unpack_position`, `pack_camera`, `send_waypoint_command`, `parse_reached_ack` |
| `utils/occupancy_grid.py` | `OccupancyGrid` class — log-odds grid, LIDAR ray casting (Bresenham), probability output |

> **Import note:** Controllers add the project root to `sys.path` so both host-side and Webots-side code can import `from utils.protocol import ...`. Webots controllers include this at the top of every controller file.

---

## Startup Order

Always start in this order to avoid connection errors:

```
1. Start Webots (opens the world, initializes controller TCP servers)
2. Start tools/visualizer.py (binds UDP port 5555, passive)
3. Start tools/camera_viewer.py (binds UDP port 5556, passive)  [optional]
4. Start planners/<your_planner>.py (connects TCP, starts sending waypoints)
```

The planner must start **after** Webots is running because the controller's TCP server opens during simulation initialization. The tools can start in any order relative to each other.

---

## File Dependency Map

```
controllers/waypoint_controller/waypoint_controller.py
  └── utils/protocol.py
  └── controllers/waypoint_controller/robot_drivers.py

planners/simple_planner.py
  └── utils/protocol.py

tools/visualizer.py
  └── utils/protocol.py
  └── utils/occupancy_grid.py

tools/camera_viewer.py
  └── utils/protocol.py
```
