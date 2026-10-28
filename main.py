#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

left_motor.control.limits(actuation=100)
right_motor.control.limits(actuation=100)


robot = DriveBase(
    left_motor=left_motor,
    right_motor=right_motor,
    wheel_diameter=40,  # Durchmesser des Antriebsrads in mm
    axle_track=200,     # Spurweite (Kettenmitte zu Kettenmitte) in mm
)


robot.settings(straight_speed=200, straight_acceleration=400)

# ---------------------------------------------------------
# TESTABLAUF
# ---------------------------------------------------------

# Startsignal
ev3.speaker.beep(frequency=500, duration=200)
ev3.screen.print("TEST START")
wait(1000)

# TEST 1: Geradeaus fahren (30 cm vor)
ev3.screen.clear()
ev3.screen.print("1. Vorwaerts")
robot.straight(300)
wait(1000)

# TEST 2: Rückwaerts fahren (30 cm zurueck)
ev3.screen.clear()
ev3.screen.print("2. Rueckwaerts")
robot.straight(-300)
wait(1000)

# TEST 3: Drehung nach links (90 Grad)
ev3.screen.clear()
ev3.screen.print("3. Drehen Links")
robot.turn(-90)
wait(1000)

# TEST 4: Drehung nach rechts (90 Grad)
ev3.screen.clear()
ev3.screen.print("4. Drehen Rechts")
robot.turn(90)
wait(1000)

# Test Ende
ev3.screen.clear()
ev3.screen.print("TEST FERTIG!")
ev3.speaker.beep(frequency=1000, duration=500)