from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Color, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait

LEFT_MOTOR_PORT = Port.A
RIGHT_MOTOR_PORT = Port.B
COLOR_SENSOR_PORT = Port.S1

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
        self.drive_base = DriveBase(
            left_motor=self.left_motor,
            right_motor=self.right_motor,
            wheel_diameter=31.3,  # in mm, including the track (calibrated)
            axle_track=200,       # track width in mm
        )
        self.drive_base.settings(straight_speed=200, straight_acceleration=400)
        self.off_band = False

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

    def straight(self, distance, speed=150):
        """distance in mm, negative = backwards"""
        self.drive_base.reset()
        direction = 1 if distance >= 0 else -1
        while abs(self.drive_base.distance()) < abs(distance):
            self.drive_base.drive(direction * speed, 0)
            self.wait(10)
        self.stop()

    def turn(self, angle, rate=90):
        """angle in degrees, positive = right"""
        self.drive_base.reset()
        direction = 1 if angle >= 0 else -1
        while abs(self.drive_base.angle()) < abs(angle):
            self.drive_base.drive(0, direction * rate)
            self.wait(10)
        self.stop()

    def show(self, *lines):
        self.ev3.screen.clear()
        for row, line in enumerate(lines):
            self.ev3.screen.draw_text(0, row * LINE_HEIGHT, line)
