"""Unified controller: teleop + LIDAR/camera/position streaming. Youbot and Pioneer."""

import sys
import os
import math
import socket
import struct

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, project_root)
sys.path.insert(0, os.path.dirname(__file__))

from controller import Supervisor, Keyboard
from robot_drivers import get_driver
from dal.protocol import (
    POSITION_PORT, CAMERA_PORT,
    pack_position, pack_camera,
    CAMERA_HEADER_SIZE
)

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_node = robot.getSelf()
robot_name = robot.getName()
driver = get_driver(robot)
keyboard = Keyboard()
keyboard.enable(timestep)

lidar = None
lidar_num_points = 0
LIDAR_NAMES = ['LDS-01', 'lidar', 'Sick LMS 291', 'lds-01']
for lname in LIDAR_NAMES:
    lidar = robot.getDevice(lname)
    if lidar is not None:
        lidar.enable(timestep)
        lidar.enablePointCloud()
        lidar_num_points = lidar.getHorizontalResolution()
        print(f"LIDAR '{lname}' enabled: {lidar_num_points} points, "
              f"FOV={math.degrees(lidar.getFov()):.0f} deg, "
              f"maxRange={lidar.getMaxRange():.1f} m")
        break
if lidar is None:
    print("No LIDAR found on this robot.")

camera = None
cam_w, cam_h = 0, 0
CAMERA_NAMES = ['camera', 'Camera', 'cam']
for cname in CAMERA_NAMES:
    camera = robot.getDevice(cname)
    if camera is not None:
        camera.enable(timestep * 4)  # lower rate to save bandwidth
        cam_w = camera.getWidth()
        cam_h = camera.getHeight()
        print(f"Camera '{cname}' enabled: {cam_w}x{cam_h}")
        break
if camera is None:
    print("No Camera found on this robot.")

pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
cam_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
VIZ_ADDR = ('localhost', POSITION_PORT)
CAM_ADDR = ('localhost', CAMERA_PORT)
MAX_CAM_UDP = 60000


def get_heading():
    ori = robot_node.getOrientation()
    return math.atan2(ori[3], ori[0])


print(f"\n=== DAL Controller [{robot_name}] ===")
print(f"Robot ID: {driver.ROBOT_ID}")
if hasattr(driver, 'strafe_left') and type(driver).__name__ == 'YoubotDriver':
    print("Keys: Arrows=move, Q/E=rotate")
else:
    print("Keys: Arrows=fwd/back/turn")
print(f"Streaming to UDP :{POSITION_PORT} (pos+lidar), :{CAMERA_PORT} (camera)")
print()

step_count = 0
cam_step = 0
while robot.step(timestep) != -1:
    step_count += 1
    pos = robot_node.getPosition()
    heading = get_heading()

    lidar_ranges = []
    if lidar is not None:
        lidar_ranges = list(lidar.getRangeImage())

    try:
        data = pack_position(driver.ROBOT_ID, pos[0], pos[1], heading, lidar_ranges)
        pos_sock.sendto(data, VIZ_ADDR)
    except Exception:
        pass

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

    if step_count % 200 == 0:
        print(f"[{robot_name}] X={pos[0]:.2f} Y={pos[1]:.2f} H={math.degrees(heading):.0f}deg "
              f"LIDAR={len(lidar_ranges)}pts")

    key = keyboard.getKey()

    if key == Keyboard.UP:
        driver.forward()
    elif key == Keyboard.DOWN:
        driver.backward()
    elif key == Keyboard.LEFT:
        driver.strafe_left()
    elif key == Keyboard.RIGHT:
        driver.strafe_right()
    elif key == ord('Q'):
        driver.turn_left()
    elif key == ord('E'):
        driver.turn_right()
    else:
        driver.stop()
