"""SLAM Viz — 2D occupancy grid + robot positions. Receives position + LIDAR via UDP from waypoint_controller."""

import sys
import os
import math
import json

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import matplotlib.colors as mcolors
import numpy as np
import socket

from utils.protocol import POSITION_PORT, unpack_position
from utils.occupancy_grid import OccupancyGrid

# ── Load world config ──────────────────────────────────────────────────────────
_world = sys.argv[1] if len(sys.argv) > 1 else 'dal-factory'
_cfg_path = os.path.join(os.path.dirname(__file__), '..', 'world_configs', f'{_world}.json')

try:
    with open(_cfg_path) as f:
        _cfg = json.load(f)
except FileNotFoundError:
    print(f"ERROR: No config found for world '{_world}' at {_cfg_path}")
    print(f"Available: dal2, dal-factory")
    sys.exit(1)

WORLD_NAME      = _cfg['name']
FLOOR_WIDTH     = _cfg['floor_width']
FLOOR_HEIGHT    = _cfg['floor_height']
FLOOR_CENTER_X  = _cfg['floor_center_x']
FLOOR_CENTER_Y  = _cfg['floor_center_y']
FIGURE_SIZE     = tuple(_cfg['figure_size'])

X_MIN = FLOOR_CENTER_X - FLOOR_WIDTH  / 2
X_MAX = FLOOR_CENTER_X + FLOOR_WIDTH  / 2
Y_MIN = FLOOR_CENTER_Y - FLOOR_HEIGHT / 2
Y_MAX = FLOOR_CENTER_Y + FLOOR_HEIGHT / 2

GRID_RESOLUTION = 0.15
LIDAR_ANGLE_MIN = math.pi
LIDAR_MAX_RANGE = _cfg.get('lidar_max_range', 3.5)

# ── Socket + occupancy grid ────────────────────────────────────────────────────
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock.bind(('localhost', POSITION_PORT))
    print(f"[DEBUG] Socket bound to UDP :{POSITION_PORT} OK")
except OSError as e:
    print(f"[DEBUG] SOCKET BIND FAILED on port {POSITION_PORT}: {e}")
    print(f"[DEBUG] Is grid_visualization.py or another tool already running on this port?")
    sys.exit(1)
sock.setblocking(False)

occ_grid = OccupancyGrid(X_MIN, X_MAX, Y_MIN, Y_MAX, resolution=GRID_RESOLUTION)

# ── Robot state from config ────────────────────────────────────────────────────
robots = {
    int(rid): {
        "name":    r['name'],
        "color":   r['color'],
        "marker":  r['marker'],
        "x": 0.0, "y": 0.0, "heading": 0.0,
        "active":  False,
    }
    for rid, r in _cfg['robots'].items()
}

# ── Figure ────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=FIGURE_SIZE)

ax.set_xlim(X_MIN - 0.3, X_MAX + 0.3)
ax.set_ylim(Y_MIN - 0.3, Y_MAX + 0.3)
ax.set_aspect('equal')

cmap = mcolors.LinearSegmentedColormap.from_list(
    'occ', [(0.2, 0.8, 0.2), (0.85, 0.85, 0.85), (0.8, 0.2, 0.2)], N=256
)
grid_img = ax.imshow(
    occ_grid.grid,
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

# ── Markers + heading lines (built from config) ────────────────────────────────
markers       = {}
heading_lines = {}
legend_elements = [
    patches.Patch(facecolor=(0.2, 0.8, 0.2),    label='Free'),
    patches.Patch(facecolor=(0.85, 0.85, 0.85), label='Unknown'),
    patches.Patch(facecolor=(0.8, 0.2, 0.2),    label='Obstacle'),
]
for rid, r in robots.items():
    fmt = r['color'][0] + r['marker']
    line, = ax.plot([], [], fmt, markersize=8, zorder=5, label=r['name'])
    hline, = ax.plot([], [], color=r['color'], linewidth=2, zorder=5)
    markers[rid]       = line
    heading_lines[rid] = hline
    legend_elements.append(
        plt.Line2D([0], [0], marker=r['marker'], color='w',
                   markerfacecolor=r['color'], markersize=8, label=r['name'])
    )

ax.legend(handles=legend_elements, loc='upper right', fontsize=7)
title_text = ax.set_title(f'{WORLD_NAME} Visualizer | Waiting for data...')

plt.tight_layout()

print(f"World: {WORLD_NAME}")
print(f"Occupancy grid: {occ_grid.width}x{occ_grid.height} cells at {GRID_RESOLUTION}m resolution")
print(f"Floor bounds: X=[{X_MIN:.2f}, {X_MAX:.2f}], Y=[{Y_MIN:.2f}, {Y_MAX:.2f}]")
print(f"Listening on UDP :{POSITION_PORT} for position + LIDAR data...")

recv_count = 0
frame_count = 0


def update(frame):
    global recv_count, frame_count
    frame_count += 1

    packets_this_frame = 0
    while True:
        try:
            data, addr = sock.recvfrom(65535)
            packets_this_frame += 1
            recv_count += 1

            # First 5 packets — print raw info
            if recv_count <= 5:
                print(f"[DEBUG] Packet #{recv_count}: {len(data)} bytes from {addr}")

            robot_id, x, y, heading, lidar_ranges = unpack_position(data)

            # First 5 packets — print parsed content
            if recv_count <= 5:
                print(f"[DEBUG]   robot_id={robot_id}  x={x:.2f}  y={y:.2f}  lidar_pts={len(lidar_ranges)}")

            if robot_id in robots:
                robots[robot_id]['x']       = x
                robots[robot_id]['y']       = y
                robots[robot_id]['heading'] = heading
                robots[robot_id]['active']  = True

                if len(lidar_ranges) > 0 and robot_id == 0:
                    inc = -(2.0 * math.pi / len(lidar_ranges))
                    occ_grid.update_from_lidar(
                        x, y, heading, lidar_ranges,
                        angle_min=LIDAR_ANGLE_MIN,
                        angle_increment=inc,
                        max_range=LIDAR_MAX_RANGE
                    )
            else:
                if recv_count <= 10:
                    print(f"[DEBUG]   robot_id={robot_id} NOT in robots dict {list(robots.keys())}")

        except BlockingIOError:
            break
        except Exception as e:
            print(f"[DEBUG] unpack error: {e}")
            break

    # Print every 20 frames if nothing is arriving
    if frame_count % 20 == 0 and recv_count == 0:
        print(f"[DEBUG] Frame {frame_count}: no UDP packets received yet — is controller running?")

    if recv_count > 0 and recv_count % 500 == 0:
        active = [
            f"{r['name']}: ({r['x']:.1f},{r['y']:.1f})"
            for r in robots.values() if r['active']
        ]
        free_pct = np.sum(occ_grid.grid < 0.3) / occ_grid.grid.size * 100
        print(f"  {' | '.join(active)} | mapped: {free_pct:.0f}% free")

    grid_img.set_data(occ_grid.grid)

    hlen = 0.3
    for rid, r in robots.items():
        if r['active']:
            markers[rid].set_xdata([r['x']])
            markers[rid].set_ydata([r['y']])
            heading_lines[rid].set_xdata([r['x'], r['x'] + hlen * math.cos(r['heading'])])
            heading_lines[rid].set_ydata([r['y'], r['y'] + hlen * math.sin(r['heading'])])

    parts = [
        f"{r['name']}: ({r['x']:.1f},{r['y']:.1f})"
        for r in robots.values() if r['active']
    ]
    title_text.set_text(
        f"{WORLD_NAME} Visualizer | {' | '.join(parts)}"
        if parts else f"{WORLD_NAME} Visualizer | Waiting for data..."
    )

    return [grid_img] + list(markers.values()) + list(heading_lines.values()) + [title_text]


ani = animation.FuncAnimation(fig, update, interval=80, blit=False, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    sock.close()
