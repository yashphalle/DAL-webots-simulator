# Adding and Using Sensors

Sensors are configured in two places: the world file (physical sensor attached to the robot) and the controller (reading sensor data in Python).

---

## Sensors Already in the Default World

| Sensor | Device name | Robot | Data |
|--------|-------------|-------|------|
| LiDAR | `LDS-01` | YouBot | 360° range scan, 3.5 m max |
| LiDAR | `Sick LMS 291` | Pioneer | 180° range scan, 8 m max |
| Camera | `camera` | Both | RGB image, configurable resolution |

These are already enabled and streamed by `dal_controller` and `waypoint_controller`.

---

## How Sensors Work in Webots

1. The sensor is physically attached to the robot in the world file (a child node)
2. In the controller, you get a handle: `sensor = robot.getDevice('device_name')`
3. Enable the sensor: `sensor.enable(timestep)` — this tells Webots to update it every `timestep` ms
4. Read data: `sensor.getRangeImage()`, `sensor.getImage()`, etc.

---

## Adding a LiDAR

### Step 1 — Add to World

1. Select the robot in Webots scene tree
2. Add a child node: `Add → PROTO nodes → Sensors → Lidar`
3. Set `name` to something like `"my_lidar"`
4. Adjust `translation` and `rotation` to position it on the robot
5. Configure `horizontalResolution`, `fov` (field of view), `maxRange`

### Step 2 — Enable in Controller

```python
lidar = robot.getDevice('my_lidar')
if lidar:
    lidar.enable(timestep)
    lidar.enablePointCloud()   # required for getRangeImage()
    print(f"LiDAR: {lidar.getHorizontalResolution()} points, FOV={math.degrees(lidar.getFov()):.0f}°")
```

### Step 3 — Read Data

```python
ranges = lidar.getRangeImage()   # list of floats, length = horizontalResolution
# ranges[i] = distance in meters for beam i
# inf = no return (open space beyond maxRange)
# nan = invalid reading
```

### Step 4 — Use for Occupancy Grid

```python
from utils.occupancy_grid import OccupancyGrid

grid = OccupancyGrid(x_min, x_max, y_min, y_max, resolution=0.15)

# In main loop:
ranges = lidar.getRangeImage()
grid.update_from_lidar(
    robot_x=x,
    robot_y=y,
    robot_heading=heading,
    ranges=ranges,
    angle_min=math.pi,      # offset if LiDAR is mounted rear-facing
    angle_increment=-(2 * math.pi / len(ranges)),  # negative = clockwise scan
    max_range=3.5
)
```

The `angle_min` and `angle_increment` depend on how the LiDAR is mounted. To find them:
- Drive the robot forward and check if the free-space in the grid matches the actual direction
- Flip the sign of `angle_increment` or add `math.pi` to `angle_min` if the grid is mirrored or rotated

---

## Adding a Camera

### Step 1 — Add to World

1. Select robot → Add child: `PROTO nodes → Sensors → Camera`
2. Set `name` to `"camera"` (or any name)
3. Set `width`, `height` (e.g., 320×240)
4. Adjust `translation` to place it on the robot

### Step 2 — Enable in Controller

```python
camera = robot.getDevice('camera')
if camera:
    camera.enable(timestep * 4)   # update every 4 timesteps to reduce CPU load
    cam_w = camera.getWidth()
    cam_h = camera.getHeight()
```

### Step 3 — Read and Stream Image

Webots returns camera images as BGRA byte arrays. Convert to RGB before streaming:

```python
from utils.protocol import pack_camera, CAMERA_PORT

img = camera.getImage()
if img:
    rgb = bytearray(cam_w * cam_h * 3)
    for px in range(cam_w * cam_h):
        src = px * 4           # BGRA
        dst = px * 3           # RGB
        rgb[dst]   = img[src + 2]   # R
        rgb[dst+1] = img[src + 1]   # G
        rgb[dst+2] = img[src]       # B

    payload = pack_camera(driver.ROBOT_ID, cam_w, cam_h, bytes(rgb))
    cam_sock.sendto(payload, ('localhost', CAMERA_PORT))
```

### Step 4 — View the Camera Feed

```bash
python tools/camera_viz.py
```

---

## Adding a Distance Sensor (Proximity/IR)

```python
# Enable
dist = robot.getDevice('ps0')   # device name from world file
dist.enable(timestep)

# Read (in main loop)
value = dist.getValue()   # voltage-like value, robot-specific range
```

Proximity sensors vary by robot model. Consult the robot's PROTO documentation for value ranges and voltage-to-distance conversion.

---

## Adding an IMU (Inertial Measurement Unit)

```python
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# Read roll, pitch, yaw in radians
roll, pitch, yaw = imu.getRollPitchYaw()
```

For 2D ground robots, `yaw` gives the robot's heading directly — simpler than computing it from the orientation matrix.

---

## Adding a GPS

```python
gps = robot.getDevice('gps')
gps.enable(timestep)

# Read position (alternative to Supervisor.getPosition())
x, y, z = gps.getValues()
```

GPS gives noisy readings in Webots (model-dependent). For precise position in studies, use `Supervisor.getSelf().getPosition()` instead, which gives exact simulation coordinates.

---

## Sensor Update Rates

`sensor.enable(N)` sets the update period to `N` milliseconds.

| Sensor | Recommended period | Why |
|--------|-------------------|-----|
| LiDAR | `timestep` (32 ms) | Navigation needs fresh obstacle data every step |
| Camera | `timestep * 4` (128 ms) | Expensive; lower rate saves CPU |
| IMU | `timestep` (32 ms) | Heading used for navigation every step |
| GPS | `timestep` (32 ms) | Position used for navigation every step |
| Distance | `timestep` (32 ms) | Reactive obstacle avoidance |

Setting a sensor to a period less than `basicTimeStep` has no effect — Webots cannot update faster than the simulation timestep.

---

## Checking if a Sensor Exists

Always check before using, especially for controllers shared between robot types:

```python
LIDAR_NAMES = ['LDS-01', 'lidar', 'Sick LMS 291']
lidar = None
for name in LIDAR_NAMES:
    lidar = robot.getDevice(name)
    if lidar is not None:
        lidar.enable(timestep)
        lidar.enablePointCloud()
        break

if lidar is None:
    print("No LiDAR found — occupancy grid disabled")
```

This pattern is already used in `waypoint_controller.py` and `dal_controller.py`.
