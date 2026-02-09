"""Teleop + position streaming. Youbot: arrows + Q/E. Pioneer: numpad 8/2/4/6."""

from controller import Supervisor, Keyboard
import struct
import socket

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

keyboard = Keyboard()
keyboard.enable(timestep)
robot_node = robot.getSelf()
robot_name = robot.getName()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
VISUALIZER_ADDRESS = ('localhost', 5555)
BASE_SPEED = 10.0

if "youBot" in robot_name:
    ROBOT_ID = 0
    wheel1 = robot.getDevice('wheel1')
    wheel2 = robot.getDevice('wheel2')
    wheel3 = robot.getDevice('wheel3')
    wheel4 = robot.getDevice('wheel4')
    wheels = [wheel1, wheel2, wheel3, wheel4]

    for w in wheels:
        w.setPosition(float('inf'))
        w.setVelocity(0.0)

    print(f"=== YOUBOT [{robot_name}] ===")
    print("Arrow UP/DOWN: Forward/Backward")
    print("Arrow LEFT/RIGHT: Strafe Left/Right")
    print("Q/E: Rotate Left/Right")

elif "Pioneer" in robot_name:
    ROBOT_ID = 1
    fl = robot.getDevice('front left wheel')
    fr = robot.getDevice('front right wheel')
    bl = robot.getDevice('back left wheel')
    br = robot.getDevice('back right wheel')
    wheels = [fl, fr, bl, br]

    for w in wheels:
        w.setPosition(float('inf'))
        w.setVelocity(0.0)
    BASE_SPEED = 5.0
    print(f"=== PIONEER 3-AT [{robot_name}] ===")
    print("Numpad 8/5: Forward/Backward")
    print("Numpad 4/6: Turn Left/Right")

else:
    ROBOT_ID = 99
    wheels = []
    print(f"Unknown robot: {robot_name}, position tracking only")

print(f"Robot ID: {ROBOT_ID}")
print(f"Sending position to visualizer on UDP :5555")
print()


def stop_all():
    for w in wheels:
        w.setVelocity(0.0)


step_count = 0
while robot.step(timestep) != -1:
    pos = robot_node.getPosition()
    try:
        sock.sendto(struct.pack('Bfff', ROBOT_ID, pos[0], pos[1], pos[2]), VISUALIZER_ADDRESS)
    except Exception:
        pass
    step_count += 1
    if step_count % 200 == 0:
        print(f"[{robot_name}] X={pos[0]:.3f}  Y={pos[1]:.3f}  Z={pos[2]:.3f}")

    key = keyboard.getKey()

    if "youBot" in robot_name:
        if key == Keyboard.UP:
            wheel1.setVelocity(BASE_SPEED)
            wheel2.setVelocity(BASE_SPEED)
            wheel3.setVelocity(BASE_SPEED)
            wheel4.setVelocity(BASE_SPEED)
        elif key == Keyboard.DOWN:
            wheel1.setVelocity(-BASE_SPEED)
            wheel2.setVelocity(-BASE_SPEED)
            wheel3.setVelocity(-BASE_SPEED)
            wheel4.setVelocity(-BASE_SPEED)
        elif key == Keyboard.LEFT:
            wheel1.setVelocity(BASE_SPEED)
            wheel2.setVelocity(-BASE_SPEED)
            wheel3.setVelocity(-BASE_SPEED)
            wheel4.setVelocity(BASE_SPEED)
        elif key == Keyboard.RIGHT:
            wheel1.setVelocity(-BASE_SPEED)
            wheel2.setVelocity(BASE_SPEED)
            wheel3.setVelocity(BASE_SPEED)
            wheel4.setVelocity(-BASE_SPEED)
        elif key == ord('Q'):
            wheel1.setVelocity(-BASE_SPEED)
            wheel2.setVelocity(BASE_SPEED)
            wheel3.setVelocity(-BASE_SPEED)
            wheel4.setVelocity(BASE_SPEED)
        elif key == ord('E'):
            wheel1.setVelocity(BASE_SPEED)
            wheel2.setVelocity(-BASE_SPEED)
            wheel3.setVelocity(BASE_SPEED)
            wheel4.setVelocity(-BASE_SPEED)
        else:
            stop_all()

    elif "Pioneer" in robot_name:
        if key == ord('8'):
            fl.setVelocity(BASE_SPEED)
            fr.setVelocity(BASE_SPEED)
            bl.setVelocity(BASE_SPEED)
            br.setVelocity(BASE_SPEED)
        elif key == ord('2') or key == ord('5'):
            fl.setVelocity(-BASE_SPEED)
            fr.setVelocity(-BASE_SPEED)
            bl.setVelocity(-BASE_SPEED)
            br.setVelocity(-BASE_SPEED)
        elif key == ord('4'):
            # Turn left (right wheels forward, left wheels backward)
            fl.setVelocity(-BASE_SPEED)
            fr.setVelocity(BASE_SPEED)
            bl.setVelocity(-BASE_SPEED)
            br.setVelocity(BASE_SPEED)
        elif key == ord('6'):
            fl.setVelocity(BASE_SPEED)
            fr.setVelocity(-BASE_SPEED)
            bl.setVelocity(BASE_SPEED)
            br.setVelocity(-BASE_SPEED)
        else:
            stop_all()
