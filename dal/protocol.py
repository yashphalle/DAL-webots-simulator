import struct

POSITION_PORT = 5555
CAMERA_PORT = 5556
COMMAND_PORT_BASE = 6000

ROBOT_YOUBOT = 0
ROBOT_PIONEER = 1

POSITION_HEADER_FMT = 'BfffH'
POSITION_HEADER_SIZE = struct.calcsize(POSITION_HEADER_FMT)


def pack_position(robot_id, x, y, heading, lidar_ranges=None):
    if lidar_ranges is None:
        lidar_ranges = []
    num = len(lidar_ranges)
    header = struct.pack(POSITION_HEADER_FMT, robot_id, x, y, heading, num)
    if num > 0:
        body = struct.pack(f'{num}f', *lidar_ranges)
        return header + body
    return header


def unpack_position(data):
    robot_id, x, y, heading, num = struct.unpack_from(POSITION_HEADER_FMT, data)
    lidar_ranges = []
    if num > 0:
        offset = POSITION_HEADER_SIZE
        lidar_ranges = list(struct.unpack_from(f'{num}f', data, offset))
    return robot_id, x, y, heading, lidar_ranges


CAMERA_HEADER_FMT = 'BHH'
CAMERA_HEADER_SIZE = struct.calcsize(CAMERA_HEADER_FMT)


def pack_camera(robot_id, width, height, rgb_bytes):
    header = struct.pack(CAMERA_HEADER_FMT, robot_id, width, height)
    return header + rgb_bytes


def unpack_camera_header(data):
    return struct.unpack_from(CAMERA_HEADER_FMT, data)


# Waypoint command protocol (TCP, text-based)
WAYPOINT_PORT = 6000

def send_waypoint_command(sock, x, y):
    """Send waypoint command to controller: 'WAYPOINT x y\\n'"""
    msg = f"WAYPOINT {x} {y}\n"
    sock.sendall(msg.encode('utf-8'))

def send_reached_ack(sock, x, y):
    """Send reached acknowledgment to planner: 'REACHED x y\\n'"""
    msg = f"REACHED {x} {y}\n"
    sock.sendall(msg.encode('utf-8'))

def parse_waypoint_command(line):
    """Parse 'WAYPOINT x y' -> (x, y) or None if invalid"""
    parts = line.strip().split()
    if len(parts) == 3 and parts[0] == "WAYPOINT":
        try:
            return (float(parts[1]), float(parts[2]))
        except ValueError:
            return None
    return None

def parse_reached_ack(line):
    """Parse 'REACHED x y' -> (x, y) or None if invalid"""
    parts = line.strip().split()
    if len(parts) == 3 and parts[0] == "REACHED":
        try:
            return (float(parts[1]), float(parts[2]))
        except ValueError:
            return None
    return None
