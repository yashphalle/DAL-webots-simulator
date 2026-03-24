# Adding a Robot

This guide covers adding a new robot type to the simulator — meaning a robot that works with the existing controller, planner, and visualizer stack.

---

## Concepts

Every robot needs three things:

1. **A PROTO node** in the world file (the physical robot with sensors)
2. **A driver class** in `robot_drivers.py` (maps high-level commands → wheel velocities)
3. **A controller** assigned in the world file (which script runs on this robot)

The existing robots are:

| Model | Driver class | ROBOT_ID | Drive type |
|-------|-------------|----------|------------|
| KUKA YouBot | `YoubotDriver` | `0` | Mecanum (omni) |
| Pioneer 3-AT | `PioneerDriver` | `1` | Differential |

---

## Step 1 — Add the Robot to the World

1. Open your `.wbt` world file in Webots
2. `Add node → PROTO nodes (Webots) → Robots → <your robot>`
3. Set `translation` to the robot's starting position
4. Set `name` to something identifiable (e.g., `"MyRobot"`)
5. Set `controller` to the controller folder name (e.g., `waypoint_controller`)
6. Save the world

The robot name must contain a unique keyword that your driver's factory function can detect (see Step 3).

---

## Step 2 — Find the Robot's Device Names

Before writing the driver, you need the exact device names Webots uses for the robot's wheels.

1. In Webots, select the robot in the scene tree
2. Expand its node to find `MotorDevice` children
3. Note the `name` field of each wheel motor

For example:
- YouBot uses `wheel1`, `wheel2`, `wheel3`, `wheel4`
- Pioneer 3-AT uses `front left wheel`, `front right wheel`, `back left wheel`, `back right wheel`

You can also find device names in the robot's PROTO file in the Webots library.

---

## Step 3 — Write a Driver Class

Add your driver to `controllers/waypoint_controller/robot_drivers.py` (and also `controllers/dal_controller/robot_drivers.py` if you want teleop support).

### Template

```python
class MyRobotDriver:

    ROBOT_ID = 2          # must be unique — determines TCP port (6000 + ROBOT_ID)
    BASE_SPEED = 6.0      # rad/s for wheel motors

    def __init__(self, robot):
        # Get wheel motor devices by their Webots name
        self.left  = robot.getDevice('left wheel motor')
        self.right = robot.getDevice('right wheel motor')
        self.wheels = [self.left, self.right]

        # Set to velocity control mode (required)
        for w in self.wheels:
            w.setPosition(float('inf'))
            w.setVelocity(0.0)

    def forward(self, speed=None):
        s = speed or self.BASE_SPEED
        self.left.setVelocity(s)
        self.right.setVelocity(s)

    def backward(self, speed=None):
        s = speed or self.BASE_SPEED
        self.left.setVelocity(-s)
        self.right.setVelocity(-s)

    def turn_left(self, speed=None):
        s = speed or self.BASE_SPEED
        self.left.setVelocity(-s)
        self.right.setVelocity(s)

    def turn_right(self, speed=None):
        s = speed or self.BASE_SPEED
        self.left.setVelocity(s)
        self.right.setVelocity(-s)

    # strafe_left/right: differential robots can only turn, not strafe
    def strafe_left(self, speed=None):
        self.turn_left(speed)

    def strafe_right(self, speed=None):
        self.turn_right(speed)

    def stop(self):
        for w in self.wheels:
            w.setVelocity(0.0)
```

### Mecanum / Omni-Directional Robots

For robots with mecanum or omnidirectional wheels, implement true lateral movement:

```python
def strafe_left(self, speed=None):
    s = speed or self.BASE_SPEED
    self.fl.setVelocity(s)
    self.fr.setVelocity(-s)
    self.bl.setVelocity(-s)
    self.br.setVelocity(s)
```

See `YoubotDriver` in the existing `robot_drivers.py` for the full mecanum implementation.

> **Adding any other robot type?** Simply add a new driver class to `robot_drivers.py` following the same interface. All driver classes must implement `forward`, `backward`, `turn_left`, `turn_right`, `strafe_left`, `strafe_right`, and `stop`. There is no base class to inherit from — just match the method signatures used by `YoubotDriver` or `PioneerDriver`.

---

## Step 4 — Register in the Factory Function

Update `get_driver()` at the bottom of `robot_drivers.py` to detect your robot:

```python
def get_driver(robot):
    name = robot.getName()
    if "youBot" in name or "Youbot" in name:
        return YoubotDriver(robot)
    elif "Pioneer" in name:
        return PioneerDriver(robot)
    elif "MyRobot" in name:            # <-- add this
        return MyRobotDriver(robot)
    else:
        raise RuntimeError(f"Unknown robot: '{name}'. Supported: Youbot, Pioneer, MyRobot.")
```

The `name` is whatever you set in the world file's `name` field.

---

## Step 5 — Update the Visualizer (optional)

If you want the visualizer to show your new robot, add it to the `robots` section of your world config file in `world_configs/<world>.json`:

```json
"robots": {
    "0": {"name": "Youbot",  "color": "red",   "marker": "o"},
    "1": {"name": "Pioneer", "color": "blue",  "marker": "s"},
    "2": {"name": "MyRobot", "color": "green", "marker": "^"}
}
```

No Python code needs to change — the visualizers build markers dynamically from the config.

---

## Step 6 — Test

1. Open the world in Webots — your robot should appear
2. Check the Webots console for `=== Waypoint Controller [MyRobot] ===` and `TCP server listening on port 6002`
3. Run a planner pointing to port `6002` (or `COMMAND_PORT_BASE + 2`)

---

## Notes on ROBOT_ID

`ROBOT_ID` must be unique across all robots running simultaneously. It determines:
- The TCP command port: `6000 + ROBOT_ID`
- The `robot_id` field in UDP packets (so the visualizer knows which robot sent the data)

Do not assign `ROBOT_ID = 0` or `1` to new robots — those are taken by YouBot and Pioneer.
