# Adding a Controller

A controller is a Python script that runs **inside Webots** as part of the simulation. It has direct access to the robot's sensors and actuators through the Webots `controller` API.

---

## Two Types of Controllers

| Type | When to use | Example |
|------|-------------|---------|
| **Teleop** | Manual keyboard control, debugging, recording trajectories | `dal_controller` |
| **Waypoint** | Receives waypoints from an external planner via TCP | `waypoint_controller` |

For most studies, you will extend or copy `waypoint_controller` since it already handles navigation and the planner protocol.

---

## Controller Directory Structure

Every controller must be in its own folder matching the folder name exactly:

```
controllers/
└── my_controller/
    ├── my_controller.py    # main file — same name as folder
    └── robot_drivers.py    # copy from an existing controller
```

Webots identifies the controller by folder name. The main `.py` file must have the same name as the folder.

> **Supporting a different robot?** If your controller needs to drive a robot not already in `robot_drivers.py`, add a new driver class there following the same interface (`forward`, `backward`, `turn_left`, `turn_right`, `strafe_left`, `strafe_right`, `stop`). See [adding_a_robot.md](adding_a_robot.md) for a full walkthrough.

---

## Step 1 — Create the Controller Folder

```bash
# from project root
cp -r controllers/waypoint_controller controllers/my_controller
mv controllers/my_controller/waypoint_controller.py controllers/my_controller/my_controller.py
```

---

## Step 2 — Write the Controller

### Minimal Controller Template

```python
"""My controller: brief description of what this controller does."""

import sys
import os
import math
import socket

# Add project root to path so utils/ is importable
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.dirname(__file__))

from controller import Supervisor
from robot_drivers import get_driver
from utils.protocol import (
    POSITION_PORT, WAYPOINT_PORT,
    pack_position,
    send_reached_ack, parse_waypoint_command
)

# --- Initialization ---
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_node = robot.getSelf()
driver = get_driver(robot)

print(f"=== My Controller [{robot.getName()}] ===")

# --- Sensor Setup ---
# (See adding_a_sensor.md for full sensor setup)
lidar = robot.getDevice('LDS-01')
if lidar:
    lidar.enable(timestep)
    lidar.enablePointCloud()

# --- UDP socket for streaming ---
pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
VIZ_ADDR = ('localhost', POSITION_PORT)

# --- Main Loop ---
while robot.step(timestep) != -1:
    pos = robot_node.getPosition()
    x, y = pos[0], pos[1]

    # Get heading from orientation matrix
    ori = robot_node.getOrientation()
    heading = math.atan2(ori[3], ori[0])

    # Stream position + LIDAR
    lidar_ranges = list(lidar.getRangeImage()) if lidar else []
    data = pack_position(driver.ROBOT_ID, x, y, heading, lidar_ranges)
    pos_sock.sendto(data, VIZ_ADDR)

    # --- Your control logic here ---
    driver.stop()
```

---

## Step 3 — Assign the Controller in the World

1. Open the world in Webots
2. Click the robot in the scene tree
3. Find the `controller` field
4. Set it to your controller folder name (e.g., `my_controller`)
5. Save the world

---

## Extending waypoint_controller

The most common pattern: copy `waypoint_controller` and add logic to it.

### Adding State to Navigation

`waypoint_controller` already has a state machine with `STATE_IDLE` and `STATE_NAVIGATING`. Add new states:

```python
STATE_IDLE = 0
STATE_NAVIGATING = 1
STATE_SCANNING = 2       # new: rotate in place to scan environment

# In the main loop, add a new branch:
elif state == STATE_SCANNING:
    rotation_steps += 1
    driver.turn_left()
    if rotation_steps >= 360 / (SPEED * timestep / 1000):
        driver.stop()
        state = STATE_IDLE
        rotation_steps = 0
```

### Reacting to Sensor Data Mid-Navigation

```python
# Inside the STATE_NAVIGATING block, before the drive command:
if lidar and min(lidar.getRangeImage()) < 0.3:
    # Obstacle detected — stop and notify planner
    driver.stop()
    state = STATE_IDLE
    if planner_conn:
        send_reached_ack(planner_conn, x, y)  # signal "done" even if not at target
```

### Sending Custom Data to Planner

The protocol is plain TCP text. To add a new message type, send it from the controller and parse it in the planner:

```python
# In controller — send a custom status message
def send_status(sock, status_string):
    msg = f"STATUS {status_string}\n"
    sock.sendall(msg.encode('utf-8'))
```

```python
# In planner — receive it
line = sock_file.readline()
if line.startswith("STATUS"):
    handle_status(line.split()[1])
```

---

## Key Webots APIs

| API | What it does |
|-----|-------------|
| `robot.step(timestep)` | Advance simulation one step. Returns `-1` when simulation ends. |
| `robot.getSelf()` | Get the robot's own Supervisor node (for position, orientation) |
| `robot_node.getPosition()` | Returns `[x, y, z]` in world coordinates |
| `robot_node.getOrientation()` | Returns 3×3 rotation matrix as 9-element list |
| `robot.getDevice('name')` | Get any sensor or actuator by its Webots name |
| `robot.getName()` | Returns the robot's `name` field from the world file |
| `robot.getBasicTimeStep()` | Returns the simulation timestep in milliseconds |

### Heading from Orientation Matrix

The orientation matrix is row-major `[R00, R01, R02, R10, R11, R12, R20, R21, R22]`.

```python
ori = robot_node.getOrientation()
heading = math.atan2(ori[3], ori[0])  # atan2(R10, R00)
```

This gives the heading in radians. For YouBot and Pioneer in the default world, forward is +Y when heading = π/2.

---

## Important Rules

- **Never import** `from controller import ...` in a planner or tool. That module only exists inside Webots.
- **Never run** a controller file directly with `python my_controller.py`. Webots executes it automatically.
- Each robot in the world can only have **one** active controller at a time.
- If the controller crashes, Webots shows an error in the console. Check the Webots console first when debugging.
