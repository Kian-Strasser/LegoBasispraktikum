from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Color, Button, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.B
COLOR_SENSOR_PORT = Port.S2
GYRO_SENSOR_PORT = Port.S1

# color of the band at the end of a section
BAND_COLOR = Color.BLUE

LINE_HEIGHT = 20


class Abort(Exception):
    """enter pressed -> back to the menu"""


class BandCrossed(Exception):
    """crossed the colored band -> next section"""


class Robot:
    """Motors, sensors and basic functions used by all sections."""

    def __init__(self):
        self.ev3 = EV3Brick()
        self.left_motor = Motor(LEFT_MOTOR_PORT)
        self.right_motor = Motor(RIGHT_MOTOR_PORT)
        self.left_motor.control.limits(actuation=100)
        self.right_motor.control.limits(actuation=100)
        self.color_sensor = ColorSensor(COLOR_SENSOR_PORT)
        # mounted upside down, so flipped: positive angle = turn to the right
        self.gyro = GyroSensor(GYRO_SENSOR_PORT, Direction.COUNTERCLOCKWISE)
        self.drive_base = DriveBase(
            left_motor=self.left_motor,
            right_motor=self.right_motor,
            wheel_diameter=31.3,  # in mm, including the track (calibrated)
            axle_track=200,       # track width in mm
        )
        self.drive_base.settings(straight_speed=200, straight_acceleration=400)
        self.off_band = False

        # the gyro calibrates at startup -> don't move the robot
        self.show("Gyro kalibriert...", "nicht bewegen!")
        wait(2000)
        self.gyro.reset_angle(0)

    def check(self):
        """Stops the current section if enter is pressed or the band is reached.

        Call it in your own loops (or just use wait()).
        """
        if Button.CENTER in self.ev3.buttons.pressed():
            raise Abort()
        if self.color_sensor.color() != BAND_COLOR:
            self.off_band = True
        elif self.off_band:
            # only counts after we were off the band, otherwise a section
            # that starts on the band would end right away
            self.off_band = False
            raise BandCrossed()

    def wait(self, ms):
        """Like wait() from pybricks, but can be aborted."""
        self.check()
        while ms > 0:
            wait(min(ms, 10))
            ms -= 10
            self.check()

    def stop(self):
        self.drive_base.stop()

    def brake(self):
        """Stops and holds the motors in place.

        stop() lets the motors coast, then the tracks relax and the robot
        rolls back a bit.
        """
        self.drive_base.stop()
        self.left_motor.hold()
        self.right_motor.hold()

    def straight(self, distance, speed=150, kp=3, brake_zone=60, min_speed=30):
        """distance in mm, negative = backwards

        Distance comes from the encoders, heading from the gyro: if the robot
        drifts off the start angle it steers back proportionally (kp). In the
        last brake_zone mm it slows down linearly to min_speed.
        Returns the gyro angle at the end (drift in degrees).
        """
        self.gyro.reset_angle(0)
        self.drive_base.reset()
        direction = 1 if distance >= 0 else -1
        while abs(self.drive_base.distance()) < abs(distance):
            remaining = abs(distance) - abs(self.drive_base.distance())
            current_speed = speed
            if remaining < brake_zone:
                current_speed = max(min_speed, speed * remaining / brake_zone)
            correction = -kp * self.gyro.angle()
            self.drive_base.drive(direction * current_speed, correction)
            self.wait(5)
        self.brake()
        self.wait(300)
        return self.gyro.angle()

    def turn(self, angle, fast=90, slow=30, slow_zone=20,
             correction_rate=15, tolerance=1, max_corrections=5):
        """angle in degrees, positive = right, measured with the gyro

        The last slow_zone degrees are turned slower to reduce overshoot.
        After that a remaining error is corrected slowly until it is within
        tolerance. Returns the gyro angle at the end.
        """
        self.gyro.reset_angle(0)
        direction = 1 if angle >= 0 else -1
        while abs(self.gyro.angle()) < abs(angle):
            remaining = abs(angle) - abs(self.gyro.angle())
            rate = slow if remaining < slow_zone else fast
            self.drive_base.drive(0, direction * rate)
            self.wait(5)
        self.brake()
        self.wait(300)  # let it settle so the final angle is right

        # turn back slowly onto the target angle
        for _ in range(max_corrections):
            error = angle - self.gyro.angle()
            if abs(error) <= tolerance:
                break
            correction = 1 if error > 0 else -1
            timer = StopWatch()
            # time limit as a safeguard in case the gyro counts the wrong way
            while (angle - self.gyro.angle()) * correction > 0 and timer.time() < 2000:
                self.drive_base.drive(0, correction * correction_rate)
                self.wait(5)
            self.brake()
            self.wait(300)
        return self.gyro.angle()

    def show(self, *lines):
        self.ev3.screen.clear()
        for row, line in enumerate(lines):
            self.ev3.screen.draw_text(0, row * LINE_HEIGHT, line)
