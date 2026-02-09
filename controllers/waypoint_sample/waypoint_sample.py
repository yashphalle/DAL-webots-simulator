"""Drive robot to a single (x, y) waypoint in world coordinates (meters)."""

from controller import Supervisor
import math

TARGET_X = 2.0
TARGET_Y = 2.0
DISTANCE_TOLERANCE = 0.15
ANGLE_TOLERANCE_RAD = 0.08
SPEED = 4.0
TURN_SPEED = 2.0

STATE_TURN = 1
STATE_GO = 2
STATE_DONE = 3

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
robot_node = robot.getSelf()
robot_name = robot.getName()

ROBOT_YOUBOT = "youbot"
ROBOT_PIONEER = "pioneer"
wheels = []
robot_type = None

if "youBot" in robot_name or "Youbot" in robot_name:
    robot_type = ROBOT_YOUBOT
    w1 = robot.getDevice('wheel1')
    w2 = robot.getDevice('wheel2')
    w3 = robot.getDevice('wheel3')
    w4 = robot.getDevice('wheel4')
    wheels = [w1, w2, w3, w4]
    print("Waypoint controller: Youbot (mecanum) detected.")
elif "Pioneer" in robot_name:
    robot_type = ROBOT_PIONEER
    fl = robot.getDevice('front left wheel')
    fr = robot.getDevice('front right wheel')
    bl = robot.getDevice('back left wheel')
    br = robot.getDevice('back right wheel')
    wheels = [fl, fr, bl, br]
    print("Waypoint controller: Pioneer 3-AT (differential) detected.")
else:
    raise RuntimeError(f"Unknown robot '{robot_name}'. Supported: Youbot, Pioneer3at.")

for w in wheels:
    w.setPosition(float('inf'))
    w.setVelocity(0.0)


def get_heading_rad():
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


def set_forward(speed):
    if robot_type == ROBOT_YOUBOT:
        w1, w2, w3, w4 = wheels
        w1.setVelocity(speed)
        w2.setVelocity(speed)
        w3.setVelocity(speed)
        w4.setVelocity(speed)
    else:
        for w in wheels:
            w.setVelocity(speed)


def set_turn(speed):
    if robot_type == ROBOT_YOUBOT:
        w1, w2, w3, w4 = wheels
        w1.setVelocity(-speed)
        w2.setVelocity(speed)
        w3.setVelocity(-speed)
        w4.setVelocity(speed)
    else:
        fl, fr, bl, br = wheels
        fl.setVelocity(-speed)
        fr.setVelocity(speed)
        bl.setVelocity(-speed)
        br.setVelocity(speed)


try:
    args = robot.getControllerArguments()
    if args and len(args) > 0:
        parts = args.split()
        if len(parts) >= 2:
            TARGET_X = float(parts[0])
            TARGET_Y = float(parts[1])
            print(f"Waypoint from controllerArgs: ({TARGET_X}, {TARGET_Y})")
except Exception:
    pass

print(f"Waypoint target (world coords): ({TARGET_X}, {TARGET_Y})")
print("Driving to waypoint...")

state = STATE_TURN

while robot.step(timestep) != -1:
    pos = robot_node.getPosition()
    x, y = pos[0], pos[1]
    dx = TARGET_X - x
    dy = TARGET_Y - y
    distance = math.sqrt(dx * dx + dy * dy)

    if distance < DISTANCE_TOLERANCE:
        stop()
        if state != STATE_DONE:
            print(f"Reached waypoint at ({x:.2f}, {y:.2f})")
        state = STATE_DONE
        continue

    desired_heading = math.atan2(dy, dx)
    current_heading = get_heading_rad()
    angle_err = angle_diff(current_heading, desired_heading)

    if state == STATE_TURN:
        if abs(angle_err) < ANGLE_TOLERANCE_RAD:
            state = STATE_GO
            set_forward(SPEED)
        else:
            set_turn(TURN_SPEED if angle_err > 0 else -TURN_SPEED)

    elif state == STATE_GO:
        if abs(angle_err) > ANGLE_TOLERANCE_RAD * 2:
            stop()
            state = STATE_TURN
        else:
            set_forward(SPEED)

print("Waypoint controller finished.")
