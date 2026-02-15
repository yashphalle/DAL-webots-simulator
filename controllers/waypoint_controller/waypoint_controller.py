"""Waypoint controller: receives waypoints via TCP, navigates robot, streams sensors."""

import sys
import os
import math
import socket
import select

# Add project root to path for imports
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.dirname(__file__))

from controller import Supervisor
from robot_drivers import get_driver
from dal.protocol import (
    POSITION_PORT, CAMERA_PORT, WAYPOINT_PORT,
    pack_position, pack_camera,
    send_reached_ack, parse_waypoint_command,
    CAMERA_HEADER_SIZE
)

# Constants
DISTANCE_TOLERANCE = 0.30  # meters
SPEED = 5.0  # m/s
TURN_GAIN = 8.0  # for differential drive
TURN_THRESHOLD = 0.4  # radians

# Navigation states
STATE_IDLE = 0
STATE_NAVIGATING = 1

# Initialize Webots
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_node = robot.getSelf()
robot_name = robot.getName()
driver = get_driver(robot)

print(f"\n=== Waypoint Controller [{robot_name}] ===")
print(f"Robot ID: {driver.ROBOT_ID}")

# Detect robot type
is_youbot = type(driver).__name__ == 'YoubotDriver'
is_pioneer = type(driver).__name__ == 'PioneerDriver'

# Setup LIDAR
lidar = None
lidar_num_points = 0
LIDAR_NAMES = ['LDS-01', 'lidar', 'Sick LMS 291', 'lds-01']
for lname in LIDAR_NAMES:
    lidar = robot.getDevice(lname)
    if lidar is not None:
        lidar.enable(timestep)
        lidar.enablePointCloud()
        lidar_num_points = lidar.getHorizontalResolution()
        print(f"LIDAR enabled: {lidar_num_points} points, FOV={math.degrees(lidar.getFov()):.0f}deg")
        break
if lidar is None:
    print("No LIDAR found.")

# Setup Camera
camera = None
cam_w, cam_h = 0, 0
CAMERA_NAMES = ['camera', 'Camera', 'cam']
for cname in CAMERA_NAMES:
    camera = robot.getDevice(cname)
    if camera is not None:
        camera.enable(timestep * 4)
        cam_w = camera.getWidth()
        cam_h = camera.getHeight()
        print(f"Camera enabled: {cam_w}x{cam_h}")
        break
if camera is None:
    print("No Camera found.")

# Setup UDP sockets for sensor streaming
pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cam_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
VIZ_ADDR = ('localhost', POSITION_PORT)
CAM_ADDR = ('localhost', CAMERA_PORT)
MAX_CAM_UDP = 60000

# Setup TCP server for waypoint commands
cmd_port = WAYPOINT_PORT + driver.ROBOT_ID
server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_sock.bind(('localhost', cmd_port))
server_sock.listen(1)
server_sock.setblocking(False)
print(f"TCP server listening on port {cmd_port}")

planner_conn = None
conn_buffer = ""  # Buffer for incomplete TCP messages


# Heading helpers
def get_heading():
    ori = robot_node.getOrientation()
    return math.atan2(ori[3], ori[0])


def angle_diff(a, b):
    """Return angle difference b - a, wrapped to [-pi, pi]"""
    d = b - a
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


# Heading calibration
print("Calibrating heading...")
pos_before = robot_node.getPosition()
heading_before = get_heading()

# Drive forward briefly
if is_youbot:
    for w in [driver.w1, driver.w2, driver.w3, driver.w4]:
        w.setVelocity(SPEED)
else:
    for w in [driver.fl, driver.fr, driver.bl, driver.br]:
        w.setVelocity(SPEED)

for _ in range(15):
    robot.step(timestep)

driver.stop()
robot.step(timestep)

pos_after = robot_node.getPosition()
dx_cal = pos_after[0] - pos_before[0]
dy_cal = pos_after[1] - pos_before[1]
actual_forward = math.atan2(dy_cal, dx_cal)
HEADING_OFFSET = angle_diff(heading_before, actual_forward)
print(f"Heading offset: {math.degrees(HEADING_OFFSET):.1f} deg")


def corrected_heading():
    return get_heading() + HEADING_OFFSET


# Mecanum drive (YouBot)
def set_mecanum(vx, vy, vrot):
    """vx=forward, vy=strafe-left, vrot=CCW rotation"""
    driver.w1.setVelocity(vx + vy - vrot)
    driver.w2.setVelocity(vx - vy + vrot)
    driver.w3.setVelocity(vx - vy - vrot)
    driver.w4.setVelocity(vx + vy + vrot)


# Differential drive (Pioneer)
def set_differential(left, right):
    driver.fl.setVelocity(left)
    driver.fr.setVelocity(right)
    driver.bl.setVelocity(left)
    driver.br.setVelocity(right)


# Navigation state
state = STATE_IDLE
target_x = None
target_y = None

print("\nWaiting for planner connection...")
print("=" * 50)

step_count = 0
cam_step = 0

# Main control loop
while robot.step(timestep) != -1:
    step_count += 1

    # Get current position and heading
    pos = robot_node.getPosition()
    x, y = pos[0], pos[1]
    heading = corrected_heading()

    # Stream position + LIDAR over UDP
    lidar_ranges = []
    if lidar is not None:
        lidar_ranges = list(lidar.getRangeImage())

    try:
        data = pack_position(driver.ROBOT_ID, x, y, heading, lidar_ranges)
        pos_sock.sendto(data, VIZ_ADDR)
    except Exception:
        pass

    # Stream camera over UDP (lower rate)
    cam_step += 1
    if camera is not None and cam_step % 4 == 0:
        try:
            img = camera.getImage()
            if img:
                raw = bytearray(cam_w * cam_h * 3)
                for px in range(cam_w * cam_h):
                    src = px * 4
                    dst = px * 3
                    raw[dst] = img[src + 2]
                    raw[dst + 1] = img[src + 1]
                    raw[dst + 2] = img[src]

                payload = pack_camera(driver.ROBOT_ID, cam_w, cam_h, bytes(raw))
                if len(payload) <= MAX_CAM_UDP:
                    cam_sock.sendto(payload, CAM_ADDR)
        except Exception:
            pass

    # Check for new planner connection (non-blocking)
    if planner_conn is None:
        readable, _, _ = select.select([server_sock], [], [], 0)
        if readable:
            planner_conn, addr = server_sock.accept()
            planner_conn.setblocking(False)
            conn_buffer = ""
            print(f"Planner connected from {addr}")

    # Check for incoming waypoint commands (non-blocking)
    if planner_conn is not None:
        try:
            readable, _, _ = select.select([planner_conn], [], [], 0)
            if readable:
                data = planner_conn.recv(1024)
                if not data:
                    # Connection closed
                    print("Planner disconnected")
                    planner_conn.close()
                    planner_conn = None
                    conn_buffer = ""
                    state = STATE_IDLE
                    driver.stop()
                else:
                    # Process received data
                    conn_buffer += data.decode('utf-8')

                    # Look for complete lines
                    while '\n' in conn_buffer:
                        line, conn_buffer = conn_buffer.split('\n', 1)
                        waypoint = parse_waypoint_command(line)
                        if waypoint:
                            target_x, target_y = waypoint
                            state = STATE_NAVIGATING
                            print(f"New waypoint: ({target_x:.2f}, {target_y:.2f})")
        except BlockingIOError:
            pass
        except Exception as e:
            print(f"TCP error: {e}")
            planner_conn.close()
            planner_conn = None
            conn_buffer = ""
            state = STATE_IDLE
            driver.stop()

    # Navigate to waypoint
    if state == STATE_NAVIGATING and target_x is not None:
        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < DISTANCE_TOLERANCE:
            # Reached waypoint
            driver.stop()
            print(f"Reached waypoint ({target_x:.2f}, {target_y:.2f})")

            # Send acknowledgment to planner
            if planner_conn is not None:
                try:
                    send_reached_ack(planner_conn, target_x, target_y)
                except Exception as e:
                    print(f"Failed to send ACK: {e}")

            state = STATE_IDLE
            target_x = None
            target_y = None
        else:
            # Drive toward waypoint
            world_angle = math.atan2(dy, dx)
            local_angle = world_angle - heading

            if is_youbot:
                # Mecanum: strafe directly toward target
                vx = SPEED * math.cos(local_angle)
                vy = SPEED * math.sin(local_angle)
                set_mecanum(vx, vy, 0)
            else:
                # Differential: turn to face target, then drive
                err = angle_diff(heading, world_angle)
                turn = max(-SPEED, min(SPEED, -TURN_GAIN * err))

                if abs(err) > TURN_THRESHOLD:
                    # Pure turning
                    set_differential(-turn, turn)
                else:
                    # Forward with arc
                    set_differential(SPEED + turn * 0.3, SPEED - turn * 0.3)

            # Progress logging
            if step_count % 100 == 0:
                print(f"  Navigating: pos=({x:.2f},{y:.2f}) dist={distance:.2f}m "
                      f"angle={math.degrees(local_angle):.0f}deg")
    elif state == STATE_IDLE:
        driver.stop()
