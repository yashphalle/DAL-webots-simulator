# Adding a World

A "world" in Webots is a `.wbt` file that defines the environment (floor, walls, objects) and which robots are present with which controllers.

---

## Option A: Create a New World from Scratch

### Step 1 — Create the `.wbt` File

1. In Webots, go to `File → New World`
2. Add a `RectangleArena` or `Floor` from `Add → PROTO nodes (Webots) → Objects → Floors`
3. Set its dimensions to match your desired environment

### Step 2 — Add Robots

See [adding_a_robot.md](adding_a_robot.md) for full instructions. In brief:
- `Add → PROTO nodes (Webots) → Robots → Kuka → KukaYouBot`
- `Add → PROTO nodes (Webots) → Robots → Adept → Pioneer3AT`

### Step 3 — Create a World Config

The visualizers load world parameters from `world_configs/<key>.json`. Create a new config file for your world:

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
        "0": {"name": "Youbot", "color": "red",  "marker": "o"}
    }
}
```

Save as `world_configs/myworld.json`.

To find floor values: in Webots, click the floor object and read its `translation` (center) and `size` (dimensions) from the scene tree.

### Step 4 — Save

Save with `File → Save World As...` into the `worlds/` directory:
```
worlds/my_world.wbt
```

---

## Option B: Duplicate an Existing World

The fastest approach for a new experiment.

1. Copy `worlds/DAL-Factory.wbt` to `worlds/my_experiment.wbt`
2. Open `my_experiment.wbt` in Webots
3. Modify the environment as needed
4. Webots will create a companion `worlds/.my_experiment.wbproj` automatically

---

## World File Structure

A `.wbt` file is a VRML-format text file. You can edit it in any text editor, but the Webots GUI is safer for structural changes. Key sections:

```
#VRML_SIM R2025a utf8

WorldInfo {
  title "My World"
  basicTimeStep 32        # simulation timestep in ms
}

Viewpoint { ... }         # initial camera position

Background { ... }        # sky color

DirectionalLight { ... }  # lighting

RectangleArena {          # floor + walls
  floorSize 7 22.5
  wallHeight 0.5
}

KukaYouBot {
  translation 0 0 0.08    # starting position
  name "KukaYouBot"
  controller "waypoint_controller"
}
```

---

## Required World Properties

For the simulator stack to work correctly, your world must have:

| Property | Why | How to set |
|----------|-----|------------|
| `WorldInfo.basicTimeStep` | Controls simulation speed and sensor update rate | Set to `32` (ms) for a good balance |
| Robot `controller` field | Determines which Python script drives the robot | Set to controller folder name (e.g., `waypoint_controller`) |
| LIDAR sensor enabled | Occupancy grid requires LIDAR | See [adding_a_sensor.md](adding_a_sensor.md) |
| Unique robot `name` fields | `robot_drivers.py` uses the name to detect type and derive ROBOT_ID from the trailing number | e.g. `"youBot_0"` → ID 0, `"youBot_1"` → ID 1 |

---

## Updating Visualizer Config for a New World

Create or edit the world's config file in `world_configs/`. See [using_visualizer.md](using_visualizer.md) for all available fields. Then run the visualizer with your config key:

```bash
python tools/slam_viz.py myworld
python tools/robot_pos_viz.py myworld
```

No Python code needs to be changed.

---

## Multi-Robot Worlds

To run multiple robots simultaneously, each robot needs:

1. A unique `name` in the world file (e.g., `"KukaYouBot"`, `"Pioneer 3-AT"`)
2. A controller assigned (`waypoint_controller` for autonomous, `dal_controller` for teleop)
3. A unique `ROBOT_ID` in its `robot_drivers.py` — this determines the TCP port

| Robot name in world file | ROBOT_ID (auto-parsed) | TCP port |
|--------------------------|------------------------|----------|
| `youBot` or `youBot_0` | `0` | `6000` |
| `youBot_1` | `1` | `6001` |
| `youBot_2` | `2` | `6002` |
| `Pioneer3at` | `1` (default) | `6001` |

ROBOT_ID is derived automatically from the trailing number in the robot's name — no code changes needed. Add the robot to the world config's `robots` dict to have it appear in the visualizer.
