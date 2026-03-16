"""Continuous planner: sends full path at once, robot follows without stopping."""

import sys
import os
import socket

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, project_root)

from utils.protocol import (
    WAYPOINT_PORT,
    send_path_command,
    parse_reached_ack
)

ROBOT_ID = 0  # 0 = YouBot, 1 = Pioneer
HOST = 'localhost'

# Full path sent at once — robot navigates continuously without stopping
WAYPOINTS = [
    (3.0, -5.0),
    (3.0,  0.0),
    (0.0,  3.0),
   (-1.0,  4.0),
   (-3.0,  0.0),
   (-3.0, -5.0),
    (0.0, -7.0),
    (3.0, -7.0),
]


def main():
    port = WAYPOINT_PORT + ROBOT_ID
    print(f"=== Continuous Planner ===")
    print(f"Connecting to controller at {HOST}:{port}...")

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, port))
        print(f"Connected!\n")
    except Exception as e:
        print(f"ERROR: Could not connect to controller: {e}")
        print(f"Make sure Webots is running with waypoint_controller")
        return

    sock_file = sock.makefile('r')

    print(f"Sending path ({len(WAYPOINTS)} waypoints):")
    for i, (x, y) in enumerate(WAYPOINTS, 1):
        print(f"  {i}. ({x:.2f}, {y:.2f})")
    print()

    try:
        send_path_command(sock, WAYPOINTS)
        print("Path sent — robot navigating continuously...")
    except Exception as e:
        print(f"ERROR: Failed to send path: {e}")
        sock.close()
        return

    # Wait for single REACHED when the last waypoint is done
    try:
        line = sock_file.readline()
        if not line:
            print("ERROR: Connection closed by controller")
        else:
            reached = parse_reached_ack(line)
            if reached:
                rx, ry = reached
                print(f"✓ Path complete, robot at ({rx:.2f}, {ry:.2f})")
            else:
                print(f"WARNING: Unexpected response: {line.strip()}")
    except Exception as e:
        print(f"ERROR: {e}")

    print("=" * 50)
    print("Done!")
    sock.close()


if __name__ == '__main__':
    main()
