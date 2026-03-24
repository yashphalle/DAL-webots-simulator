# Using the Visualizers

There are two visualization tools. Both are passive — they only listen, never send commands.

| Tool | Command | What it shows |
|------|---------|---------------|
| `slam_viz.py` | `python tools/slam_viz.py <world>` | 2D LIDAR occupancy grid + robot positions + headings |
| `robot_pos_viz.py` | `python tools/robot_pos_viz.py <world>` | Robot position dots on a plain floor grid |

`<world>` is the config key: `dal-factory` or `dal2`. Defaults to `dal2` if omitted.

---

## Quick Start

```bash
# SLAM occupancy grid (DAL-Factory world)
python tools/slam_viz.py dal-factory

# Robot positions only (DAL-Factory world)
python tools/robot_pos_viz.py dal-factory

# DAL2 world
python tools/slam_viz.py dal2
python tools/robot_pos_viz.py dal2
```

Both tools bind UDP port 5555. **Run only one at a time** — the second will fail to bind the port.

---

## What slam_viz Shows

| Visual | Meaning |
|--------|---------|
| Green cells | Free space (LIDAR beam passed through) |
| Gray cells | Unknown (not yet observed) |
| Red cells | Obstacle (LIDAR beam hit something) |
| Coloured dot + line | Robot position and heading direction |
| Black rectangle | World floor boundary |

## What robot_pos_viz Shows

| Visual | Meaning |
|--------|---------|
| Coloured dot | Robot position |
| Light gray lines | Grid overlay |
| Black rectangle | World floor boundary |

---

## How slam_viz Works

```
Controller (Webots)
    |
    | UDP port 5555
    | Packet: robot_id, x, y, heading, lidar_ranges[]
    |
    v
tools/slam_viz.py
    |
    ├── Updates robot x/y/heading
    └── Calls OccupancyGrid.update_from_lidar()  [robot_id == 0 only]
            |
            └── Bresenham ray casting → log-odds update → displayed via matplotlib imshow
```

Only `robot_id = 0` contributes to the occupancy grid. Other robots show position markers but their LIDAR does not update the map. This is intentional — see `slam_viz.py` line:
```python
if len(lidar_ranges) > 0 and robot_id == 0:
```

---

## World Config

Both tools load world parameters from `world_configs/<world>.json` instead of hardcoded constants. To add a new world or change bounds, create or edit a config file:

```json
{
    "name": "MyWorld",
    "floor_width": 10.0,
    "floor_height": 10.0,
    "floor_center_x": 0.0,
    "floor_center_y": 0.0,
    "grid_resolution": 0.25,
    "lidar_max_range": 4.0,
    "figure_size": [8, 8],
    "robots": {
        "0": {"name": "Youbot", "color": "red",  "marker": "o"},
        "1": {"name": "Robot2", "color": "blue", "marker": "s"}
    }
}
```

Save as `world_configs/myworld.json`, then run:
```bash
python tools/slam_viz.py myworld
```

### Config fields

| Field | Description |
|-------|-------------|
| `floor_width` | X extent of the floor in meters |
| `floor_height` | Y extent of the floor in meters |
| `floor_center_x/y` | Center of floor in world coordinates |
| `grid_resolution` | Meters per grid cell in `slam_viz` (0.15 default) |
| `lidar_max_range` | Max range cast per beam — match your LIDAR's `maxRange` |
| `figure_size` | matplotlib figure `[width, height]` in inches |
| `robots` | Dict keyed by robot ID string — name, color, marker shape |

### Current configs

**`world_configs/dal-factory.json`** — DAL-Factory world (21.2 × 21.0 m, 3 robots)

**`world_configs/dal2.json`** — DAL2 world (6.95 × 22.54 m, 2 robots)

---

## LIDAR Angle Parameters

The LDS-01 on the YouBot scans clockwise starting from behind the robot. This is corrected by:

```python
LIDAR_ANGLE_MIN = math.pi    # beam 0 points backward
inc = -(2.0 * math.pi / len(lidar_ranges))  # clockwise = negative increment
```

If you add a different LIDAR, you may need to adjust these. Symptoms of wrong parameters:
- Grid builds in the wrong direction relative to robot motion
- Obstacles appear on the opposite side from where they actually are

---

## Saving the Grid

To save the current occupancy grid to a file:

```python
import numpy as np
np.save('occupancy_grid.npy', occ_grid.grid)   # probability array [0,1]

# Load later:
data = np.load('occupancy_grid.npy')
```

To save as an image:
```python
import matplotlib.pyplot as plt
plt.imsave('map.png', occ_grid.grid, cmap='RdYlGn_r', vmin=0, vmax=1)
```

---
