# Adding a Planner

A planner is a Python script that runs on the **host machine** (outside Webots). It connects to the controller via TCP, sends waypoints, and waits for acknowledgments.

---

## The Planner-Controller Protocol

```
Planner                         Controller (inside Webots)
  |                                     |
  |----TCP connect to port 6000-------->|
  |                                     |
  |----"WAYPOINT -1.0 2.0\n"---------->| robot starts navigating
  |                                     |
  |<---"REACHED -1.0 2.0\n"------------| robot arrived
  |                                     |
  |----"WAYPOINT 3.0 5.0\n"---------->| next waypoint
  |                                     |
  ...
```

- Port = `6000 + ROBOT_ID` (`6000` for YouBot, `6001` for Pioneer)
- Text-based, newline-terminated messages
- Synchronous: send one waypoint, wait for `REACHED`, then send the next

---

## Step 1 — Create the Planner File

```bash
# from project root
cp planners/simple_planner.py planners/my_planner.py
```

---

## Step 2 — Write the Planner

### Minimal Template

```python
"""My planner: brief description of the mission logic."""

import sys
import os
import socket

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, project_root)

from utils.protocol import (
    WAYPOINT_PORT,
    send_waypoint_command,
    parse_reached_ack
)

ROBOT_ID = 0   # 0 = YouBot, 1 = Pioneer
HOST = 'localhost'


def connect(robot_id, host='localhost'):
    port = WAYPOINT_PORT + robot_id
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print(f"Connected to robot {robot_id} on port {port}")
    return sock, sock.makefile('r')


def send_and_wait(sock, sock_file, x, y):
    """Send waypoint and block until REACHED."""
    send_waypoint_command(sock, x, y)
    while True:
        line = sock_file.readline()
        if not line:
            raise ConnectionError("Controller disconnected")
        result = parse_reached_ack(line)
        if result:
            return result


def main():
    sock, sock_file = connect(ROBOT_ID)

    # Define your mission as a list of (x, y) waypoints
    waypoints = [
        (1.0, 2.0),
        (3.0, 4.0),
        (0.0, 0.0),
    ]

    for x, y in waypoints:
        print(f"Going to ({x}, {y})...")
        rx, ry = send_and_wait(sock, sock_file, x, y)
        print(f"Reached ({rx:.2f}, {ry:.2f})")

    sock.close()


if __name__ == '__main__':
    main()
```

---

## Step 3 — Run the Planner

```bash
python planners/my_planner.py
```

Make sure Webots is running first with `waypoint_controller` assigned to the robot.

---

## Multi-Robot Planner

To control multiple robots simultaneously, open one TCP connection per robot and use threads:

```python
import threading

def run_robot(robot_id, waypoints):
    sock, sock_file = connect(robot_id)
    for x, y in waypoints:
        send_and_wait(sock, sock_file, x, y)
    sock.close()

youbot_waypoints  = [(1.0, 2.0), (3.0, 0.0)]
pioneer_waypoints = [(2.0, 3.0), (0.0, 1.0)]

t0 = threading.Thread(target=run_robot, args=(0, youbot_waypoints))
t1 = threading.Thread(target=run_robot, args=(1, pioneer_waypoints))

t0.start()
t1.start()
t0.join()
t1.join()
```

Each robot runs independently. There is no synchronization between them in the base planner — add that yourself if needed (e.g., a `threading.Barrier` to make robots wait for each other at certain waypoints).

---

## Waypoint Sources

Planners can get waypoints from many sources. Common patterns:

### Hardcoded List
```python
waypoints = [(1.0, 2.0), (3.0, 4.0)]
```

### From a YAML/JSON Config File
```python
import json
with open('configs/mission.json') as f:
    config = json.load(f)
waypoints = [(wp['x'], wp['y']) for wp in config['waypoints']]
```

### From an Algorithm (e.g., Grid-Based Search)
```python
# Compute next waypoint dynamically after each REACHED
while not mission_complete():
    next_wp = compute_next_waypoint(current_position, occupancy_grid)
    rx, ry = send_and_wait(sock, sock_file, *next_wp)
    update_state(rx, ry)
```

### From a File Updated in Real Time
```python
import ast
# Another process writes waypoints to a file
with open('live_waypoints.txt') as f:
    waypoints = [ast.literal_eval(line) for line in f]
```

---

## Error Handling

Always handle disconnections gracefully:

```python
try:
    rx, ry = send_and_wait(sock, sock_file, x, y)
except ConnectionError as e:
    print(f"Lost connection to robot: {e}")
    # Decide: retry, abort, or continue with next robot
```

Common failure modes:
- Controller crashes inside Webots — TCP connection drops
- Webots simulation paused — controller stops sending, planner blocks on `readline()`
- Robot stuck, never reaches waypoint — planner blocks forever

For studies where robots might get stuck, add a timeout:
```python
sock.settimeout(30.0)   # raise socket.timeout after 30s of no response
```

---

## What a Planner Should NOT Do

- Import or use `from controller import ...` — that's Webots-only
- Block indefinitely without a timeout in production code
- Send the next waypoint before receiving `REACHED` — the controller ignores waypoints that arrive while navigating
