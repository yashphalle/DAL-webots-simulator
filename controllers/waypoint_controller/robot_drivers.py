class YoubotDriver:

    BASE_SPEED = 10.0

    def __init__(self, robot, robot_id=0):
        self.ROBOT_ID = robot_id
        self.w1 = robot.getDevice('wheel1')
        self.w2 = robot.getDevice('wheel2')
        self.w3 = robot.getDevice('wheel3')
        self.w4 = robot.getDevice('wheel4')
        self.wheels = [self.w1, self.w2, self.w3, self.w4]
        for w in self.wheels:
            w.setPosition(float('inf'))
            w.setVelocity(0.0)

    def forward(self, speed=None):
        s = speed or self.BASE_SPEED
        for w in self.wheels:
            w.setVelocity(s)

    def backward(self, speed=None):
        s = speed or self.BASE_SPEED
        for w in self.wheels:
            w.setVelocity(-s)

    def strafe_left(self, speed=None):
        s = speed or self.BASE_SPEED
        self.w1.setVelocity(s)
        self.w2.setVelocity(-s)
        self.w3.setVelocity(-s)
        self.w4.setVelocity(s)

    def strafe_right(self, speed=None):
        s = speed or self.BASE_SPEED
        self.w1.setVelocity(-s)
        self.w2.setVelocity(s)
        self.w3.setVelocity(s)
        self.w4.setVelocity(-s)

    def turn_left(self, speed=None):
        s = speed or self.BASE_SPEED
        self.w1.setVelocity(-s)
        self.w2.setVelocity(s)
        self.w3.setVelocity(-s)
        self.w4.setVelocity(s)

    def turn_right(self, speed=None):
        s = speed or self.BASE_SPEED
        self.w1.setVelocity(s)
        self.w2.setVelocity(-s)
        self.w3.setVelocity(s)
        self.w4.setVelocity(-s)

    def stop(self):
        for w in self.wheels:
            w.setVelocity(0.0)


class PioneerDriver:

    BASE_SPEED = 5.0

    def __init__(self, robot, robot_id=1):
        self.ROBOT_ID = robot_id
        self.fl = robot.getDevice('front left wheel')
        self.fr = robot.getDevice('front right wheel')
        self.bl = robot.getDevice('back left wheel')
        self.br = robot.getDevice('back right wheel')
        self.wheels = [self.fl, self.fr, self.bl, self.br]
        for w in self.wheels:
            w.setPosition(float('inf'))
            w.setVelocity(0.0)

    def forward(self, speed=None):
        s = speed or self.BASE_SPEED
        for w in self.wheels:
            w.setVelocity(s)

    def backward(self, speed=None):
        s = speed or self.BASE_SPEED
        for w in self.wheels:
            w.setVelocity(-s)

    def strafe_left(self, speed=None):
        self.turn_left(speed)

    def strafe_right(self, speed=None):
        self.turn_right(speed)

    def turn_left(self, speed=None):
        s = speed or self.BASE_SPEED
        self.fl.setVelocity(-s)
        self.fr.setVelocity(s)
        self.bl.setVelocity(-s)
        self.br.setVelocity(s)

    def turn_right(self, speed=None):
        s = speed or self.BASE_SPEED
        self.fl.setVelocity(s)
        self.fr.setVelocity(-s)
        self.bl.setVelocity(s)
        self.br.setVelocity(-s)

    def stop(self):
        for w in self.wheels:
            w.setVelocity(0.0)


def get_driver(robot):
    import re
    name = robot.getName()
    m = re.search(r'(\d+)$', name)
    robot_id = int(m.group(1)) if m else None

    if "youBot" in name or "Youbot" in name:
        return YoubotDriver(robot, robot_id if robot_id is not None else 0)
    elif "Pioneer" in name:
        return PioneerDriver(robot, robot_id if robot_id is not None else 1)
    else:
        raise RuntimeError(f"Unknown robot: '{name}'. Supported: Youbot, Pioneer3at.")
