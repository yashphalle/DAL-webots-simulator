# Using the Visualizer

The visualizer is a host-side tool that displays a live 2D occupancy grid built from LIDAR data, along with robot positions and headings. It receives data over UDP and requires no changes to the simulation.

---

## Quick Start

From the project root:
```bash
python tools/visualizer.py
```

A matplotlib window opens. It stays blank (all gray = unknown) until a robot with LIDAR starts streaming data.

---

## What It Shows

| Visual | Meaning |
|--------|---------|
| Green cells | Free space (LIDAR beam passed through) |
| Gray cells | Unknown (not yet observed) |
| Red cells | Obstacle (LIDAR beam hit something) |
| Red dot + line | YouBot position and heading direction |
| Blue square + line | Pioneer position and heading direction |
| Black rectangle | World floor boundary |

---

## How It Works

```
Controller (Webots)
    |
    | UDP port 5555
    | Packet: robot_id, x, y, heading, lidar_ranges[]
    |
    v
tools/visualizer.py
    |
    ├── Updates robot x/y/heading in robots dict
    └── Calls OccupancyGrid.update_from_lidar()
            |
            └── Bresenham ray casting → log-odds update
                    |
                    └── Displayed via matplotlib imshow
```

The visualizer drains all pending UDP packets every 80 ms animation frame. Multiple robots can stream simultaneously; each packet contains a `robot_id` to route it correctly.

Currently only `robot_id = 0` (YouBot) contributes to the occupancy grid. Pioneer position is displayed but its LIDAR does not update the grid. This is set at line 116 of `tools/visualizer.py`:
```python
if len(lidar_ranges) > 0 and robot_id == 0:
```
Change `robot_id == 0` to `robot_id == 1` or remove the condition to use Pioneer's LIDAR instead (or both).

---

## Grid Configuration

The grid is initialized with these parameters in `tools/visualizer.py`:

```python
FLOOR_WIDTH    = 6.95    # X extent of the floor in meters
FLOOR_HEIGHT   = 22.54   # Y extent of the floor in meters
FLOOR_CENTER_X = 0.0     # X center of floor in world coordinates
FLOOR_CENTER_Y = 0.14    # Y center of floor in world coordinates
GRID_RESOLUTION = 0.15   # meters per grid cell
```

If you change the world, update these to match. See [adding_a_world.md](adding_a_world.md).

### Grid Size

The number of grid cells is:
```
width  = ceil(FLOOR_WIDTH / GRID_RESOLUTION)  = ceil(6.95 / 0.15) = 47
height = ceil(FLOOR_HEIGHT / GRID_RESOLUTION) = ceil(22.54 / 0.15) = 151
```

Total: 47 × 151 = 7,097 cells. The visualizer prints this on startup:
```
Occupancy grid: 47x151 cells at 0.15m resolution
```

Smaller resolution = finer detail = more memory and slower rendering. For large worlds, `0.20` or `0.25` m is a practical resolution.

---

## LIDAR Angle Parameters

The LDS-01 on the YouBot scans clockwise (negative angle increment) and is mounted facing the rear of the robot. This is corrected by:

```python
LIDAR_ANGLE_MIN = math.pi    # offset: beam 0 points backward
inc = -(2.0 * math.pi / len(lidar_ranges))  # clockwise = negative increment
```

If you add a different LIDAR or mount it differently, you may need to adjust these. Symptoms of wrong parameters:
- Grid builds in the wrong direction relative to robot motion
- Obstacles appear on opposite side from where they actually are

To calibrate: drive the robot forward and verify that the free space appears ahead of the robot in the visualizer.

---

## Using grid_visualization.py (Simple Alternative)

`tools/grid_visualization.py` is a simpler visualization tool that:
- Shows robot positions on a plain grid overlay
- Does **not** process LIDAR data or build an occupancy map
- Useful for debugging position streaming without the computational cost of ray casting

```bash
python tools/grid_visualization.py
```

Use this with `youbot_dal` controller (position-only streaming) or for quick position verification.

---

## Extending the Visualizer

### Add a Third Robot

1. Add an entry to the `robots` dict:
```python
robots = {
    0: {"name": "Youbot",  "color": "red",   "x": 0.0, "y": 0.0, "heading": 0.0, "active": False},
    1: {"name": "Pioneer", "color": "blue",  "x": 0.0, "y": 0.0, "heading": 0.0, "active": False},
    2: {"name": "MyRobot", "color": "green", "x": 0.0, "y": 0.0, "heading": 0.0, "active": False},
}
```

2. Add matplotlib markers for the new robot (after the existing marker definitions):
```python
myrobot_marker, = ax.plot([], [], 'g^', markersize=8, zorder=5, label='MyRobot')
myrobot_heading, = ax.plot([], [], 'g-', linewidth=2, zorder=5)
```

3. In the `update()` function, add rendering for robot 2:
```python
if robots[2]["active"]:
    r = robots[2]
    myrobot_marker.set_xdata([r["x"]])
    myrobot_marker.set_ydata([r["y"]])
    hlen = 0.3
    myrobot_heading.set_xdata([r["x"], r["x"] + hlen * math.cos(r["heading"])])
    myrobot_heading.set_ydata([r["y"], r["y"] + hlen * math.sin(r["heading"])])
```

4. Return the new artists from `update()`:
```python
return grid_img, youbot_marker, ..., myrobot_marker, myrobot_heading, title_text
```

### Per-Robot Occupancy Grids

To maintain a separate map per robot, create one `OccupancyGrid` instance per robot and route each UDP packet to the correct grid:

```python
grids = {
    0: OccupancyGrid(X_MIN, X_MAX, Y_MIN, Y_MAX, resolution=GRID_RESOLUTION),
    1: OccupancyGrid(X_MIN, X_MAX, Y_MIN, Y_MAX, resolution=GRID_RESOLUTION),
}

# In update():
if len(lidar_ranges) > 0 and robot_id in grids:
    grids[robot_id].update_from_lidar(x, y, heading, lidar_ranges, ...)
```

Then display them side-by-side using `plt.subplots(1, 2)` instead of a single `ax`.

---

## Saving the Grid

To save the current occupancy grid to a file:

```python
import numpy as np

# After the grid is built:
np.save('occupancy_grid.npy', occ_grid.grid)   # save probability array

# Load later:
data = np.load('occupancy_grid.npy')
```

To save as an image:
```python
import matplotlib.pyplot as plt
plt.imsave('map.png', occ_grid.grid, cmap='RdYlGn_r', vmin=0, vmax=1)
```

---
