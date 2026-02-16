"""Simple Mavic controller: takeoff and hover at a fixed point."""

from controller import Robot
import sys
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class MavicController(Robot):
    K_VERTICAL_THRUST = 70.0
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 2.0
    K_ROLL_P = 40.0
    K_PITCH_P = 25.0

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")

        camera_pitch = self.getDevice("camera pitch")
        if camera_pitch:
            camera_pitch.setPosition(0.7)

        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.target_altitude = 1.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.step_count = 0
        self.prev_altitude = None
        self.prev_x = None
        self.prev_y = None

    def run(self):
        while self.step(self.time_step) != -1:
            self.step_count += 1
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()

            dx = self.target_x - x_pos
            dy = self.target_y - y_pos
            distance = np.sqrt(dx * dx + dy * dy)

            if self.prev_x is not None and self.prev_y is not None:
                dt = self.time_step * 0.001
                vx = (x_pos - self.prev_x) / dt
                vy = (y_pos - self.prev_y) / dt
                vx_body = vx * np.cos(yaw) + vy * np.sin(yaw)
                vy_body = -vx * np.sin(yaw) + vy * np.cos(yaw)
                damp_xy = 2.0
                pitch_damp = clamp(-vx_body * damp_xy, -0.3, 0.3)
                roll_damp = clamp(-vy_body * damp_xy, -0.3, 0.3)
            else:
                pitch_damp = 0.0
                roll_damp = 0.0
            self.prev_x = x_pos
            self.prev_y = y_pos

            altitude_error = self.target_altitude - altitude
            if self.prev_altitude is not None:
                dt = self.time_step * 0.001
                vertical_velocity = (altitude - self.prev_altitude) / dt
                damping = clamp(-vertical_velocity * 3.0, -8, 8)
            else:
                damping = 0.0
            self.prev_altitude = altitude

            clamped_error = clamp(altitude_error + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = clamp(self.K_VERTICAL_P * clamped_error + damping, -10, 10)

            yaw_target = np.arctan2(dy, dx)
            yaw_error = yaw_target - yaw
            while yaw_error > np.pi:
                yaw_error -= 2 * np.pi
            while yaw_error < -np.pi:
                yaw_error += 2 * np.pi

            if distance > 0.05:
                yaw_disturbance = clamp(yaw_error * 0.4, -0.4, 0.4)
            else:
                yaw_disturbance = clamp(yaw_error * 0.2, -0.2, 0.2)

            K_XY = 0.5
            TILT_MAX_XY = 0.55
            dx_body = dx * np.cos(yaw) + dy * np.sin(yaw)
            dy_body = -dx * np.sin(yaw) + dy * np.cos(yaw)
            pitch_disturbance = clamp(K_XY * dx_body + pitch_damp, -TILT_MAX_XY, TILT_MAX_XY)
            roll_disturbance = clamp(-K_XY * dy_body + roll_damp, -TILT_MAX_XY, TILT_MAX_XY)

            roll_input = clamp(self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance, -20, 20)
            pitch_input = clamp(self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance, -20, 20)
            yaw_input = clamp(yaw_disturbance, -5, 5)

            base_thrust = self.K_VERTICAL_THRUST + vertical_input
            
            front_left = base_thrust - yaw_input + pitch_input - roll_input
            front_right = base_thrust + yaw_input + pitch_input + roll_input
            rear_left = base_thrust + yaw_input - pitch_input - roll_input
            rear_right = base_thrust - yaw_input - pitch_input + roll_input

            front_left = clamp(front_left, 1, 576)
            front_right = clamp(front_right, 1, 576)
            rear_left = clamp(rear_left, 1, 576)
            rear_right = clamp(rear_right, 1, 576)

            self.front_left_motor.setVelocity(front_left)
            self.front_right_motor.setVelocity(-front_right)
            self.rear_left_motor.setVelocity(-rear_left)
            self.rear_right_motor.setVelocity(rear_right)

            if self.step_count % 50 == 0:
                vvel = (altitude - self.prev_altitude) / (self.time_step * 0.001) if self.step_count > 1 else 0
                print(f"  Pos: ({x_pos:.2f}, {y_pos:.2f}) target: ({self.target_x:.2f}, {self.target_y:.2f}) "
                      f"dist: {distance:.2f} Alt: {altitude:.2f}m dx: {dx:.2f} dy: {dy:.2f}")


robot = MavicController()
robot.run()
