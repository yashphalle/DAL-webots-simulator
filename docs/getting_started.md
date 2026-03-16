# Getting Started

## Prerequisites

### 1. Webots R2025a

Download and install from [cyberbotics.com](https://cyberbotics.com/).

> **Exact version required.** The world files and PROTO references are pinned to R2025a. Other versions may fail to load the world.

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

## Quick Start: Teleoperation Mode

Run the simulator with manual keyboard control. No planner needed.

### Step 1 — Open the World

1. Open Webots
2. `File → Open World...`
3. Navigate to `worlds/DAL.wbt`
4. Click the **Play** button (▶)

You should see a factory floor with a KUKA YouBot and a Pioneer 3-AT.

### Step 2 — Start the Visualizer

In a terminal from the project root:
```bash
python tools/visualizer.py
```

A matplotlib window opens showing the 2D occupancy grid. It will start blank (gray) until a robot moves.

### Step 3 — Drive the Robot

Click on the 3D Webots viewport to give it keyboard focus, then:

**YouBot (orange robot):**
| Key | Action |
|-----|--------|
| `↑` | Forward |
| `↓` | Backward |
| `←` | Strafe left |
| `→` | Strafe right |
| `Q` | Rotate left |
| `E` | Rotate right |

**Pioneer (gray robot):**
| Key | Action |
|-----|--------|
| `↑` | Forward |
| `↓` | Backward |
| `←` | Turn left |
| `→` | Turn right |

As you drive, the visualizer builds the occupancy grid from LIDAR data in real time.

### Step 4 — View Camera (optional)

In a second terminal:
```bash
python tools/camera_viewer.py
```

Press `q` in the camera window to close it.

---

## Quick Start: Autonomous Waypoint Mode

Run the simulator with the planner sending waypoints automatically.

### Step 1 — Open the World

Same as above, but assign `waypoint_controller` to the robot:

1. In Webots, right-click the YouBot
2. `Edit → Select`
3. In the scene tree, find the `controller` field
4. Set it to `waypoint_controller`
5. Click **Play** (▶)

The controller will print `TCP server listening on port 6000` in the Webots console.

### Step 2 — Start the Visualizer (optional)

```bash
python tools/visualizer.py
```

### Step 3 — Run the Planner

```bash
python planners/simple_planner.py
```

Expected output:
```
=== Simple Planner ===
Connecting to controller at localhost:6000...
Connected!

Waypoint list (4 waypoints):
  1. (-1.00, 2.00)
  2. (-1.00, 8.00)
  3. (1.00, 5.00)
  4. (2.00, 3.00)

[1/4] Sending waypoint: (-1.00, 2.00)
  Waiting for robot to reach waypoint...
  ✓ Robot reached (-1.00, 2.00)
...
```

---

## Verifying Everything Works

After running the full stack:

- Webots console: `Planner connected from ('127.0.0.1', ...)` and periodic position logs
- Visualizer: robot marker moves, grid fills in with green (free) and red (obstacle) cells
- Planner terminal: successive `✓ Robot reached` messages

---
