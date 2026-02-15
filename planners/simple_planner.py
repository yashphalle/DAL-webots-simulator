"""Simple planner: sends hardcoded waypoints one-at-a-time to controller."""

import sys
import os
import socket

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, project_root)

from dal.protocol import (
    WAYPOINT_PORT,
    send_waypoint_command,
    parse_reached_ack
)

ROBOT_ID = 0  # 0 = YouBot, 1 = Pioneer
HOST = 'localhost'

# Hardcoded waypoint list
WAYPOINTS = [
    (-1.0, 2.0),
    (-1.0, 8.0),
    (1.0, 5.0),
    (2.0, 3.0),
]


def main():
    port = WAYPOINT_PORT + ROBOT_ID
    print(f"=== Simple Planner ===")
    print(f"Connecting to controller at {HOST}:{port}...")

    # Connect to controller
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((HOST, port))
        print(f"Connected!\n")
    except Exception as e:
        print(f"ERROR: Could not connect to controller: {e}")
        print(f"Make sure Webots is running with waypoint_controller")
        return

    sock_file = sock.makefile('r')

    print(f"Waypoint list ({len(WAYPOINTS)} waypoints):")
    for i, (x, y) in enumerate(WAYPOINTS, 1):
        print(f"  {i}. ({x:.2f}, {y:.2f})")
    print()

    # Send waypoints one at a time
    for i, (x, y) in enumerate(WAYPOINTS, 1):
        print(f"[{i}/{len(WAYPOINTS)}] Sending waypoint: ({x:.2f}, {y:.2f})")

        # Send waypoint command
        try:
            send_waypoint_command(sock, x, y)
        except Exception as e:
            print(f"ERROR: Failed to send waypoint: {e}")
            break

        # Wait for "REACHED" acknowledgment (blocking read)
        print(f"  Waiting for robot to reach waypoint...")
        try:
            line = sock_file.readline()
            if not line:
                print(f"ERROR: Connection closed by controller")
                break

            reached = parse_reached_ack(line)
            if reached:
                rx, ry = reached
                print(f"  ✓ Robot reached ({rx:.2f}, {ry:.2f})")
            else:
                print(f"  WARNING: Unexpected response: {line.strip()}")
        except Exception as e:
            print(f"ERROR: Failed to receive acknowledgment: {e}")
            break

        print()

    print("=" * 50)
    print("All waypoints completed!")
    sock.close()


if __name__ == '__main__':
    main()
