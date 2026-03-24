# Getting Started

## Prerequisites

### 1. Webots R2025a

Download and install from [cyberbotics.com](https://cyberbotics.com/).

The world files and PROTO references are pinned to R2025a. Other versions may fail to load the world.

Supported platforms: Linux, Windows, macOS.

### 2. Python 3.7+

Verify with:
```bash
python --version
```

### 3. Python Dependencies

From the project root:
```bash
pip install -r requirements.txt
```

This installs: `numpy`, `matplotlib`, `opencv-python`.

---

## Quick Start: Autonomous Mode

### Step 1 — Open the World

1. Open Webots
2. `File → Open World...`
3. Navigate to `worlds/DAL-Factory.wbt`
4. Click the **Play** button (▶)

You should see a factory floor with three KUKA YouBots. The Webots console will print:
```
TCP server listening on port 6000
TCP server listening on port 6001
TCP server listening on port 6002
```

### Step 2 — Start the Visualizers

Open two terminals from the project root:

```bash
# Terminal 1 — SLAM occupancy grid
python tools/slam_viz.py dal-factory

# Terminal 2 — Camera feed (optional)
python tools/camera_viz.py
```

- `slam_viz` opens a matplotlib window showing the 2D occupancy grid — starts blank (gray) until robots move
- `camera_viz` shows the live camera feed from each robot — press `q` to close

> **Note:** `slam_viz` and `robot_pos_viz` both use UDP port 5555 — run only one at a time.

### Step 3 — Run the Planner

```bash
# Terminal 3
python planners/continuous_planner.py
```

The robot navigates the full path continuously without stopping at each waypoint.

Expected output:
```
=== Continuous Planner ===
Connecting to controller at localhost:6000...
Connected!

Sending path (8 waypoints):
  1. (3.00, -5.00)
  2. (3.00, 0.00)
  3. (0.00, 3.00)
  4. (-1.00, 4.00)
  5. (-3.00, 0.00)
  6. (-3.00, -5.00)
  7. (0.00, -7.00)
  8. (3.00, -7.00)

Path sent — robot navigating continuously...
✓ Path complete, robot at (3.00, -7.00)
```

---

## Alternative: Simple Waypoint Mode

The simple planner sends one waypoint at a time and waits for the robot to arrive before sending the next.

```bash
python planners/simple_planner.py
```

Expected output:
```
=== Simple Planner ===
Connecting to controller at localhost:6000...
Connected!

[1/8] Sending waypoint: (3.00, -5.00)
  Waiting for robot to reach waypoint...
  ✓ Robot reached (3.00, -5.00)

[2/8] Sending waypoint: (3.00, 0.00)
  Waiting for robot to reach waypoint...
  ✓ Robot reached (3.00, 0.00)
...
```

---

## Alternative: Position-Only Visualizer

If you don't need the LIDAR occupancy grid and just want to see robot positions:

```bash
python tools/robot_pos_viz.py dal-factory
```

Lighter weight than `slam_viz` — no ray casting, just dots on a floor plan.

---

## Verifying Everything Works

After running the full stack:

- **Webots console:** `Planner connected from ('127.0.0.1', ...)` and periodic position logs
- **slam_viz:** robot marker moves, grid fills in with green (free) and red (obstacle) cells
- **camera_viz:** live camera window appears, updates as robot moves
- **Planner terminal:** `✓ Path complete` at the end

---

## DAL2 World

The DAL2 world is a narrower corridor environment with one YouBot and one Pioneer 3-AT.

```bash
# Open worlds/DAL2.wbt in Webots, then:
python tools/slam_viz.py dal2
python tools/camera_viz.py
python planners/continuous_planner.py
```

---
