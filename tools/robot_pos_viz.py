
import sys
import os
import json
import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np
import socket
import struct

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
GRID_RESOLUTION = _cfg['grid_resolution']
FIGURE_SIZE     = tuple(_cfg['figure_size'])

X_MIN = FLOOR_CENTER_X - FLOOR_WIDTH  / 2
X_MAX = FLOOR_CENTER_X + FLOOR_WIDTH  / 2
Y_MIN = FLOOR_CENTER_Y - FLOOR_HEIGHT / 2
Y_MAX = FLOOR_CENTER_Y + FLOOR_HEIGHT / 2

# ── Robot definitions from config ─────────────────────────────────────────────
_robot_cfg = _cfg['robots']
robots = {
    int(rid): {
        "name":   r['name'],
        "color":  r['color'],
        "marker": r['marker'],
        "x": 0.0, "y": 0.0,
        "active": False,
    }
    for rid, r in _robot_cfg.items()
}

# ── UDP socket ────────────────────────────────────────────────────────────────
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('localhost', 5555))
sock.setblocking(False)

# ── Figure ────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=FIGURE_SIZE)

ax.set_xlim(X_MIN - 0.5, X_MAX + 0.5)
ax.set_ylim(Y_MIN - 0.5, Y_MAX + 0.5)
ax.set_aspect('equal')

floor_rect = patches.Rectangle(
    (X_MIN, Y_MIN), FLOOR_WIDTH, FLOOR_HEIGHT,
    linewidth=2, edgecolor='black', facecolor='#f0f0f0'
)
ax.add_patch(floor_rect)

for x in np.arange(X_MIN, X_MAX + GRID_RESOLUTION, GRID_RESOLUTION):
    ax.axvline(x, color='lightgray', linewidth=0.3)
for y in np.arange(Y_MIN, Y_MAX + GRID_RESOLUTION, GRID_RESOLUTION):
    ax.axhline(y, color='lightgray', linewidth=0.3)

boundary = patches.Rectangle(
    (X_MIN, Y_MIN), FLOOR_WIDTH, FLOOR_HEIGHT,
    linewidth=3, edgecolor='black', facecolor='none'
)
ax.add_patch(boundary)

ax.set_xlabel('X (meters)')
ax.set_ylabel('Y (meters)')

# ── Legend + markers (built from config) ──────────────────────────────────────
legend_elements = [patches.Patch(facecolor='#f0f0f0', edgecolor='black', label='Floor')]
markers = {}
for rid, r in robots.items():
    fmt = r['color'][0] + r['marker']
    line, = ax.plot([], [], fmt, markersize=10, zorder=5, label=r['name'])
    markers[rid] = line
    legend_elements.append(
        plt.Line2D([0], [0], marker=r['marker'], color='w',
                   markerfacecolor=r['color'], markersize=10, label=r['name'])
    )

ax.legend(handles=legend_elements, loc='upper right')
title_text = ax.set_title(f'{WORLD_NAME} | Waiting for robots...')

plt.tight_layout()

print(f"World: {WORLD_NAME}")
print(f"Floor bounds: X=[{X_MIN:.2f}, {X_MAX:.2f}], Y=[{Y_MIN:.2f}, {Y_MAX:.2f}]")
print(f"Grid resolution: {GRID_RESOLUTION}m")
print("Listening on UDP :5555 for robot positions...")
for rid, r in robots.items():
    print(f"  Robot ID {rid} ({r['name']}) = {r['color']} {r['marker']}")

recv_count = 0


def update(frame):
    global recv_count

    while True:
        try:
            data, _ = sock.recvfrom(4096)
            robot_id, x, y, _ = struct.unpack_from('Bfff', data)
            if robot_id in robots:
                robots[robot_id]['x'] = x
                robots[robot_id]['y'] = y
                robots[robot_id]['active'] = True

            recv_count += 1
            if recv_count % 400 == 0:
                parts = [
                    f"{r['name']}: ({r['x']:.2f}, {r['y']:.2f})"
                    for r in robots.values() if r['active']
                ]
                print(f"  {' | '.join(parts)}")
        except BlockingIOError:
            break
        except Exception:
            break

    for rid, r in robots.items():
        if r['active']:
            markers[rid].set_xdata([r['x']])
            markers[rid].set_ydata([r['y']])

    parts = [
        f"{r['name']}: ({r['x']:.2f}, {r['y']:.2f})"
        for r in robots.values() if r['active']
    ]
    title_text.set_text(f"{WORLD_NAME} | {' | '.join(parts)}" if parts else f"{WORLD_NAME} | Waiting for robots...")

    return list(markers.values()) + [title_text]


ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    sock.close()
