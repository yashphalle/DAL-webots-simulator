from controller import Supervisor
import math

WAYPOINTS = [
    (-1.0, 2.0),
    (-1.0, 8.0),
]

DISTANCE_TOLERANCE = 0.30
SPEED = 5.0
TURN_GAIN = 8.0

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_node = robot.getSelf()
robot_name = robot.getName()

wheels = [
    robot.getDevice('front left wheel'),
    robot.getDevice('front right wheel'),
    robot.getDevice('back left wheel'),
    robot.getDevice('back right wheel'),
]
print(f"Waypoint Pioneer: [{robot_name}]")

for w in wheels:
    w.setPosition(float('inf'))
    w.setVelocity(0.0)


def get_heading():
    ori = robot_node.getOrientation()
    return math.atan2(ori[3], ori[0])


def angle_diff(a, b):
    d = b - a
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


def stop():
    for w in wheels:
        w.setVelocity(0.0)


def set_differential(left, right):
    wheels[0].setVelocity(left)
    wheels[1].setVelocity(right)
    wheels[2].setVelocity(left)
    wheels[3].setVelocity(right)


# --- Heading calibration ---
print("Calibrating heading...")
pos_before = robot_node.getPosition()
heading_before = get_heading()
for w in wheels:
    w.setVelocity(SPEED)
for _ in range(15):
    robot.step(timestep)
stop()
robot.step(timestep)
pos_after = robot_node.getPosition()
dx_cal = pos_after[0] - pos_before[0]
dy_cal = pos_after[1] - pos_before[1]
actual_forward = math.atan2(dy_cal, dx_cal)
HEADING_OFFSET = angle_diff(heading_before, actual_forward)
print(f"  Heading offset: {math.degrees(HEADING_OFFSET):.1f} deg")


def corrected_heading():
    return get_heading() + HEADING_OFFSET


# --- Navigation ---
wp_index = 0
step_count = 0

print(f"\nWaypoints: {WAYPOINTS}")
print(f"Starting with waypoint {wp_index}: {WAYPOINTS[wp_index]}")

while robot.step(timestep) != -1:
    step_count += 1
    target_x, target_y = WAYPOINTS[wp_index]

    pos = robot_node.getPosition()
    x, y = pos[0], pos[1]
    dx = target_x - x
    dy = target_y - y
    distance = math.sqrt(dx * dx + dy * dy)

    if distance < DISTANCE_TOLERANCE:
        stop()
        print(f"Reached waypoint {wp_index} at ({x:.2f}, {y:.2f})")
        wp_index = (wp_index + 1) % len(WAYPOINTS)
        print(f"Next waypoint {wp_index}: {WAYPOINTS[wp_index]}")
        continue

    world_angle = math.atan2(dy, dx)
    heading = corrected_heading()
    err = angle_diff(heading, world_angle)

    # Pioneer turn: fl=-s, fr=+s = turn left (from robot_drivers.py)
    # So set_differential(-turn, +turn) where positive turn = turn left
    turn = max(-SPEED, min(SPEED, TURN_GAIN * err))

    if abs(err) > 0.4:
        set_differential(-turn, turn)
    else:
        set_differential(SPEED - turn * 0.3, SPEED + turn * 0.3)

    if step_count % 100 == 0:
        print(f"  wp={wp_index} pos=({x:.2f},{y:.2f}) dist={distance:.2f} "
              f"heading={math.degrees(heading):.0f} desired={math.degrees(world_angle):.0f} "
              f"err={math.degrees(err):.1f}")
