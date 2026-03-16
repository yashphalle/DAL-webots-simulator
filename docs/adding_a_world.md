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

### Step 3 — Configure Floor Bounds

The visualizer and occupancy grid need to know the world floor bounds. These are currently hardcoded in `tools/visualizer.py`:

```python
FLOOR_WIDTH = 6.95
FLOOR_HEIGHT = 22.54
FLOOR_CENTER_X = 0.0
FLOOR_CENTER_Y = 0.14
```

Update these to match your new world. The values are in meters and define the rectangular region the grid covers:
- `FLOOR_WIDTH`: total X extent
- `FLOOR_HEIGHT`: total Y extent
- `FLOOR_CENTER_X`, `FLOOR_CENTER_Y`: center of the floor in world coordinates

To find these values: in Webots, click the floor object, and read its `translation` (center) and `size` (dimensions) from the scene tree.

### Step 4 — Save

Save with `File → Save World As...` into the `worlds/` directory:
```
worlds/my_world.wbt
```

---

## Option B: Duplicate an Existing World

The fastest approach for a new experiment.

1. Copy `worlds/DAL.wbt` to `worlds/my_experiment.wbt`
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
| Unique robot `name` fields | `robot_drivers.py` uses the name to detect robot type | `"KukaYouBot"` for YouBot, `"Pioneer 3-AT"` for Pioneer |

---

## Updating Visualizer Floor Bounds for a New World

If your new world has different floor dimensions than `DAL.wbt`, update `tools/visualizer.py`:

```python
# Find these lines near the top and update:
FLOOR_WIDTH = <your_floor_x_size>
FLOOR_HEIGHT = <your_floor_y_size>
FLOOR_CENTER_X = <your_floor_translation_x>
FLOOR_CENTER_Y = <your_floor_translation_y>
```

Also update `OccupancyGrid` resolution if needed:
```python
GRID_RESOLUTION = 0.15  # meters per cell — smaller = more detail, more memory
```

---

## Multi-Robot Worlds

To run multiple robots simultaneously, each robot needs:

1. A unique `name` in the world file (e.g., `"KukaYouBot"`, `"Pioneer 3-AT"`)
2. A controller assigned (`waypoint_controller` for autonomous, `dal_controller` for teleop)
3. A unique `ROBOT_ID` in its `robot_drivers.py` — this determines the TCP port

| Robot | ROBOT_ID | TCP port |
|-------|----------|----------|
| YouBot | `0` | `6000` |
| Pioneer | `1` | `6001` |

The visualizer already handles both robots simultaneously — no changes needed.
