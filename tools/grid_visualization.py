
import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import numpy as np
import socket
import struct

FLOOR_WIDTH = 6.95
FLOOR_HEIGHT = 22.54
FLOOR_CENTER_X = 0.0
FLOOR_CENTER_Y = 0.14

X_MIN = FLOOR_CENTER_X - FLOOR_WIDTH / 2
X_MAX = FLOOR_CENTER_X + FLOOR_WIDTH / 2
Y_MIN = FLOOR_CENTER_Y - FLOOR_HEIGHT / 2
Y_MAX = FLOOR_CENTER_Y + FLOOR_HEIGHT / 2
GRID_RESOLUTION = 0.25

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('localhost', 5555))
sock.setblocking(False)

robots = {
    0: {"name": "Youbot", "color": "red", "x": 0.0, "y": 0.0, "active": False},
    1: {"name": "Pioneer", "color": "blue", "x": 0.0, "y": 0.0, "active": False},
}

fig, ax = plt.subplots(figsize=(6, 18))

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

# Legend
legend_elements = [
    patches.Patch(facecolor='#f0f0f0', edgecolor='black', label='Floor'),
    plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red',
               markersize=10, label='Youbot'),
    plt.Line2D([0], [0], marker='s', color='w', markerfacecolor='blue',
               markersize=10, label='Pioneer'),
]
ax.legend(handles=legend_elements, loc='upper right')
youbot_marker, = ax.plot([], [], 'ro', markersize=10, zorder=5)
pioneer_marker, = ax.plot([], [], 'bs', markersize=10, zorder=5)
title_text = ax.set_title('DAL World - 2D Grid | Waiting for robots...')

plt.tight_layout()

print(f"Floor bounds: X=[{X_MIN:.2f}, {X_MAX:.2f}], Y=[{Y_MIN:.2f}, {Y_MAX:.2f}]")
print("Listening on UDP :5555 for robot positions...")
print("  Robot ID 0 (Youbot)  = Red circle")
print("  Robot ID 1 (Pioneer) = Blue square")

recv_count = 0


def update(frame):
    """Called by FuncAnimation each frame. Drains UDP buffer and updates markers."""
    global recv_count

    # Drain all pending UDP packets
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            robot_id, x, y, z = struct.unpack('Bfff', data)
            if robot_id in robots:
                robots[robot_id]["x"] = x
                robots[robot_id]["y"] = y
                robots[robot_id]["active"] = True

            recv_count += 1
            if recv_count % 400 == 0:
                parts = []
                for rid, r in robots.items():
                    if r["active"]:
                        parts.append(f"{r['name']}: ({r['x']:.2f}, {r['y']:.2f})")
                print(f"  {' | '.join(parts)}")
        except BlockingIOError:
            break
        except Exception:
            break

    # Update markers
    if robots[0]["active"]:
        youbot_marker.set_xdata([robots[0]["x"]])
        youbot_marker.set_ydata([robots[0]["y"]])

    if robots[1]["active"]:
        pioneer_marker.set_xdata([robots[1]["x"]])
        pioneer_marker.set_ydata([robots[1]["y"]])

    parts = []
    for rid, r in robots.items():
        if r["active"]:
            parts.append(f"{r['name']}: ({r['x']:.2f}, {r['y']:.2f})")
    if parts:
        title_text.set_text(f"DAL World | {' | '.join(parts)}")

    return youbot_marker, pioneer_marker, title_text


ani = animation.FuncAnimation(fig, update, interval=50, blit=True, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    print("\nStopped by user")
finally:
    sock.close()
