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
from utils.protocol import (
    POSITION_PORT, CAMERA_PORT, WAYPOINT_PORT,
    pack_position, pack_camera,
    send_reached_ack, parse_waypoint_command, parse_path_command,
    CAMERA_HEADER_SIZE
)

# Constants
DISTANCE_TOLERANCE = 0.30  # meters
TURN_GAIN = 8.0
TURN_THRESHOLD = 0.4  # radians

# Navigation states
STATE_IDLE = 0
STATE_NAVIGATING = 1
STATE_PATH_FOLLOWING = 2  # continuous path following with pure pursuit

# Pure pursuit constants
LOOKAHEAD_DISTANCE = 0.3     # meters — how far ahead on path to aim for
WAYPOINT_SWITCH_DIST = 0.3   # meters — how close before advancing to next waypoint

# Initialize Webots
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_node = robot.getSelf()
robot_name = robot.getName()
driver = get_driver(robot)
SPEED = driver.BASE_SPEED  # YouBot=10.0, Pioneer=5.0

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

# Continuous path following state
path = []
path_idx = 0
last_lookahead_idx = -1  # track lookahead changes for debug prints
last_phase = ""           # track phase changes for debug prints


def get_lookahead_point(robot_x, robot_y):
    """Return (point, index) of first path point at least LOOKAHEAD_DISTANCE from robot, or the last point."""
    for i in range(path_idx, len(path)):
        dx = path[i][0] - robot_x
        dy = path[i][1] - robot_y
        if math.sqrt(dx * dx + dy * dy) >= LOOKAHEAD_DISTANCE:
            return path[i], i
    return path[-1], len(path) - 1

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
    if camera is not None and cam_step % 16 == 0:
        try:
            img = camera.getImage()
            if img is not None and len(img) > 0:
                bgra = bytes(img)
                r = bgra[2::4]
                g = bgra[1::4]
                b = bgra[0::4]
                raw = bytes(val for rgb in zip(r, g, b) for val in rgb)
                payload = pack_camera(driver.ROBOT_ID, cam_w, cam_h, raw)
                if len(payload) <= MAX_CAM_UDP:
                    cam_sock.sendto(payload, CAM_ADDR)
                else:
                    if cam_step == 16:
                        print(f"[CAM] Payload too large: {len(payload)} bytes (max {MAX_CAM_UDP}) — reduce camera resolution")
            else:
                if cam_step == 16:
                    print(f"[CAM] getImage() returned empty/None — camera may not be rendering")
        except Exception as e:
            if cam_step <= 32:
                print(f"[CAM] Exception sending camera: {e}")

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
                        new_path = parse_path_command(line)
                        if new_path:
                            path = new_path
                            path_idx = 0
                            last_lookahead_idx = -1
                            state = STATE_PATH_FOLLOWING
                            print(f"[PATH] Received {len(path)} waypoints, starting at wp 1: {path[0]}")
                            continue
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
                    set_differential(turn, -turn)
                else:
                    # Forward with arc
                    set_differential(SPEED + turn * 0.3, SPEED - turn * 0.3)

            # Progress logging
            if step_count % 100 == 0:
                print(f"  Navigating: pos=({x:.2f},{y:.2f}) dist={distance:.2f}m "
                      f"angle={math.degrees(local_angle):.0f}deg")
    elif state == STATE_PATH_FOLLOWING and path:
        cx, cy = path[path_idx]
        dist_to_current = math.sqrt((cx - x) ** 2 + (cy - y) ** 2)

        # Final waypoint — stop when within tolerance
        if path_idx == len(path) - 1 and dist_to_current < DISTANCE_TOLERANCE:
            driver.stop()
            print(f"[DONE] Path complete at ({x:.2f}, {y:.2f})")
            if planner_conn is not None:
                try:
                    send_reached_ack(planner_conn, path[-1][0], path[-1][1])
                except Exception as e:
                    print(f"Failed to send ACK: {e}")
            state = STATE_IDLE
            path = []
            path_idx = 0
            last_lookahead_idx = -1
        else:
            # Advance path index when close enough — no stopping
            if dist_to_current < WAYPOINT_SWITCH_DIST and path_idx < len(path) - 1:
                path_idx += 1
                last_lookahead_idx = -1  # force lookahead print on next step
                last_phase = ""           # force phase print on next step
                print(f"[SWITCH] wp {path_idx} → wp {path_idx + 1}: "
                      f"now targeting ({path[path_idx][0]:.2f},{path[path_idx][1]:.2f}) "
                      f"at pos=({x:.2f},{y:.2f})")

            # Steer toward lookahead point
            (lx, ly), la_idx = get_lookahead_point(x, y)
            if la_idx != last_lookahead_idx:
                last_lookahead_idx = la_idx
                la_dist = math.sqrt((lx - x) ** 2 + (ly - y) ** 2)
                print(f"[LOOK] Lookahead → wp {la_idx + 1} ({lx:.2f},{ly:.2f}) | dist={la_dist:.2f}m")

            # Phase label — print only when phase changes
            if path_idx == len(path) - 1:
                phase = "FINAL APPROACH"
            elif la_idx > path_idx:
                phase = f"CURVING  (looking ahead to wp {la_idx + 1} while passing wp {path_idx + 1})"
            else:
                phase = f"APPROACHING wp {path_idx + 1} ({path[path_idx][0]:.2f},{path[path_idx][1]:.2f})"
            if phase != last_phase:
                last_phase = phase
                print(f"[PHASE] {phase}")

            dx = lx - x
            dy = ly - y
            world_angle = math.atan2(dy, dx)
            local_angle = world_angle - heading

            if is_youbot:
                vx = SPEED * math.cos(local_angle)
                vy = SPEED * math.sin(local_angle)
                set_mecanum(vx, vy, 0)
            else:
                err = angle_diff(heading, world_angle)
                turn = max(-SPEED, min(SPEED, -TURN_GAIN * err))
                if abs(err) > TURN_THRESHOLD:
                    set_differential(turn, -turn)
                else:
                    set_differential(SPEED + turn * 0.3, SPEED - turn * 0.3)
    elif state == STATE_IDLE:
        driver.stop()
