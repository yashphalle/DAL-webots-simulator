# DAL Simulator Package

---

## Repository Structure

```
DAL/
├── worlds/
│   └── DAL.wbt               # Webots world (factory, robots, sensors)
├── controllers/
│   ├── dal_controller/       # Main: teleop + LIDAR/camera/position streaming
│   ├── youbot_dal/           # teleop + position-only streaming
│   └── waypoint_sample/     # Drive to (x,y) waypoint in world frame
├── dal/                      # Shared library (protocol, occupancy grid)
│   ├── protocol.py           # Ports, packet formats (position, camera)
│   └── occupancy_grid.py     # Log-odds grid, LIDAR ray casting, cell freeze
└── tools/
    ├── visualizer.py         # 2D occupancy grid + robot positions (UDP)
    └── camera_viewer.py     # Live camera window (UDP)
```

- **World**: `worlds/DAL.wbt` 
- **Controllers**: Assigned in the world file; run inside Webots. Use `dal_controller`
- **dal**: Protocol and occupancy grid used by both controller (pack) and tools (unpack, display).
- **tools**: Run on the host; receive UDP from Webots controllers.



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
| 6000+  | TCP command  |

---
