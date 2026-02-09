"""2D occupancy grid + robot positions. Receives position + LIDAR via UDP from dal_controller."""

import sys
import os
import math

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import numpy as np
import socket

from dal.protocol import POSITION_PORT, unpack_position
from dal.occupancy_grid import OccupancyGrid

FLOOR_WIDTH = 6.95
FLOOR_HEIGHT = 22.54
FLOOR_CENTER_X = 0.0
FLOOR_CENTER_Y = 0.14

X_MIN = FLOOR_CENTER_X - FLOOR_WIDTH / 2
X_MAX = FLOOR_CENTER_X + FLOOR_WIDTH / 2
Y_MIN = FLOOR_CENTER_Y - FLOOR_HEIGHT / 2
Y_MAX = FLOOR_CENTER_Y + FLOOR_HEIGHT / 2

GRID_RESOLUTION = 0.15

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('localhost', POSITION_PORT))
sock.setblocking(False)

occ_grid = OccupancyGrid(X_MIN, X_MAX, Y_MIN, Y_MAX, resolution=GRID_RESOLUTION)

robots = {
    0: {"name": "Youbot", "color": "red", "x": 0.0, "y": 0.0, "heading": 0.0, "active": False},
    1: {"name": "Pioneer", "color": "blue", "x": 0.0, "y": 0.0, "heading": 0.0, "active": False},
}

LIDAR_ANGLE_MIN = math.pi
LIDAR_MAX_RANGE = 3.5

fig, ax = plt.subplots(figsize=(7, 20))

ax.set_xlim(X_MIN - 0.3, X_MAX + 0.3)
ax.set_ylim(Y_MIN - 0.3, Y_MAX + 0.3)
ax.set_aspect('equal')

cmap = mcolors.LinearSegmentedColormap.from_list(
    'occ', [(0.2, 0.8, 0.2), (0.85, 0.85, 0.85), (0.8, 0.2, 0.2)], N=256
)
grid_img = ax.imshow(    occ_grid.grid,
    extent=[X_MIN, X_MAX, Y_MIN, Y_MAX],
    origin='lower',
    cmap=cmap,
    vmin=0.0, vmax=1.0,
    alpha=0.7,
    interpolation='nearest'
)

boundary = patches.Rectangle(
    (X_MIN, Y_MIN), FLOOR_WIDTH, FLOOR_HEIGHT,
    linewidth=2, edgecolor='black', facecolor='none', zorder=3
)
ax.add_patch(boundary)

ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')

youbot_marker, = ax.plot([], [], 'ro', markersize=8, zorder=5, label='Youbot')
pioneer_marker, = ax.plot([], [], 'bs', markersize=8, zorder=5, label='Pioneer')
youbot_heading_line, = ax.plot([], [], 'r-', linewidth=2, zorder=5)
pioneer_heading_line, = ax.plot([], [], 'b-', linewidth=2, zorder=5)

legend_elements = [
    patches.Patch(facecolor=(0.2, 0.8, 0.2), label='Free'),
    patches.Patch(facecolor=(0.85, 0.85, 0.85), label='Unknown'),
    patches.Patch(facecolor=(0.8, 0.2, 0.2), label='Obstacle'),
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=8, label='Youbot'),
    plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='blue', markersize=8, label='Pioneer'),
]
ax.legend(handles=legend_elements, loc='upper right', fontsize=7)

title_text = ax.set_title('DAL Visualizer | Waiting for data...')

plt.tight_layout()

print(f"Occupancy grid: {occ_grid.width}x{occ_grid.height} cells at {GRID_RESOLUTION}m resolution")
print(f"Floor bounds: X=[{X_MIN:.2f}, {X_MAX:.2f}], Y=[{Y_MIN:.2f}, {Y_MAX:.2f}]")
print(f"Listening on UDP :{POSITION_PORT} for position + LIDAR data...")

recv_count = 0


def update(frame):
    global recv_count

    # Drain all pending packets
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            robot_id, x, y, heading, lidar_ranges = unpack_position(data)

            if robot_id in robots:
                robots[robot_id]["x"] = x
                robots[robot_id]["y"] = y
                robots[robot_id]["heading"] = heading
                robots[robot_id]["active"] = True

                # Update occupancy grid with LIDAR data
                if len(lidar_ranges) > 0 and robot_id == 0:
                    inc = -(2.0 * math.pi / len(lidar_ranges))
                    occ_grid.update_from_lidar(
                        x, y, heading, lidar_ranges,
                        angle_min=LIDAR_ANGLE_MIN,
                        angle_increment=inc,
                        max_range=LIDAR_MAX_RANGE
                    )

            recv_count += 1
            if recv_count % 500 == 0:
                active = [f"{r['name']}: ({r['x']:.1f},{r['y']:.1f})" for r in robots.values() if r['active']]
                free_pct = np.sum(occ_grid.grid < 0.3) / occ_grid.grid.size * 100
                print(f"  {' | '.join(active)} | mapped: {free_pct:.0f}% free")

        except BlockingIOError:
            break
        except Exception as e:
            print(f"  Error: {e}")
            break

    grid_img.set_data(occ_grid.grid)

    if robots[0]["active"]:
        r = robots[0]
        youbot_marker.set_xdata([r["x"]])
        youbot_marker.set_ydata([r["y"]])
        hlen = 0.3
        youbot_heading_line.set_xdata([r["x"], r["x"] + hlen * math.cos(r["heading"])])
        youbot_heading_line.set_ydata([r["y"], r["y"] + hlen * math.sin(r["heading"])])

    if robots[1]["active"]:
        r = robots[1]
        pioneer_marker.set_xdata([r["x"]])
        pioneer_marker.set_ydata([r["y"]])
        hlen = 0.3
        pioneer_heading_line.set_xdata([r["x"], r["x"] + hlen * math.cos(r["heading"])])
        pioneer_heading_line.set_ydata([r["y"], r["y"] + hlen * math.sin(r["heading"])])

    parts = []
    for r in robots.values():
        if r["active"]:
            parts.append(f"{r['name']}: ({r['x']:.1f},{r['y']:.1f})")
    if parts:
        title_text.set_text(f"DAL Visualizer | {' | '.join(parts)}")

    return grid_img, youbot_marker, pioneer_marker, youbot_heading_line, pioneer_heading_line, title_text


ani = animation.FuncAnimation(fig, update, interval=80, blit=True, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    sock.close()
