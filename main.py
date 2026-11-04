#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase


# This program requires LEGO EV3 MicroPython v2.0 or higher.


ev3 = EV3Brick()

left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

left_motor.control.limits(actuation=100)
right_motor.control.limits(actuation=100)

# Gyro ist kopfueber verbaut und zaehlt sonst gegen die Lenkrichtung
# -> umdrehen, damit positiver Winkel = Drehung nach rechts
gyro = GyroSensor(Port.S1, Direction.COUNTERCLOCKWISE)


robot = DriveBase(
    left_motor=left_motor,
    right_motor=right_motor,
    wheel_diameter=31.3,  # effektiver Durchmesser inkl. Kette in mm (kalibriert: 40 mm -> 235 statt 300 mm)
    axle_track=200,     # Spurweite (Kettenmitte zu Kettenmitte) in mm
)

robot.settings(straight_speed=200, straight_acceleration=400)


def brake():
    """Stoppt und haelt die Motoren aktiv in Position.

    robot.stop() laesst die Motoren frei auslaufen, dann entspannen sich
    die Ketten und der Roboter rutscht ein Stueck zurueck.
    """
    robot.stop()
    left_motor.hold()
    right_motor.hold()


def gyro_straight(distance, speed=150, kp=3, brake_zone=60, min_speed=30):
    """Faehrt distance mm geradeaus (negativ = rueckwaerts).

    Strecke ueber die Encoder, Richtung ueber den Gyro: weicht der Roboter
    vom Startwinkel ab, wird proportional (kp) gegengelenkt. In den letzten
    brake_zone mm wird linear bis min_speed abgebremst.
    """
    gyro.reset_angle(0)
    robot.reset()
    direction = 1 if distance >= 0 else -1
    while abs(robot.distance()) < abs(distance):
        remaining = abs(distance) - abs(robot.distance())
        current_speed = speed
        if remaining < brake_zone:
            current_speed = max(min_speed, speed * remaining / brake_zone)
        correction = -kp * gyro.angle()
        robot.drive(direction * current_speed, correction)
        wait(5)
    brake()
    wait(300)
    return gyro.angle()


def gyro_turn(angle, fast=90, slow=30, slow_zone=20,
              correction_rate=15, tolerance=1, max_corrections=5):
    """Dreht um angle Grad (positiv = rechts) gemessen mit dem Gyro.

    Die letzten slow_zone Grad wird langsamer gedreht, um Ueberschwingen
    zu verringern. Danach wird ein verbleibender Fehler (Ueber- oder
    Unterdrehen) langsam korrigiert, bis er innerhalb von tolerance liegt.
    """
    gyro.reset_angle(0)
    direction = 1 if angle >= 0 else -1
    while abs(gyro.angle()) < abs(angle):
        remaining = abs(angle) - abs(gyro.angle())
        rate = slow if remaining < slow_zone else fast
        robot.drive(0, direction * rate)
        wait(5)
    brake()
    wait(300)  # Nachlaufen abwarten, damit der Endwinkel stimmt

    # Nachkorrektur: langsam auf den Zielwinkel zurueckdrehen
    for _ in range(max_corrections):
        error = angle - gyro.angle()
        if abs(error) <= tolerance:
            break
        correction = 1 if error > 0 else -1
        timer = StopWatch()
        # Zeitgrenze als Sicherung, falls der Gyro falsch herum zaehlt
        while (angle - gyro.angle()) * correction > 0 and timer.time() < 2000:
            robot.drive(0, correction * correction_rate)
            wait(5)
        brake()
        wait(300)
    return gyro.angle()


def show(*lines):
    ev3.screen.clear()
    for line in lines:
        ev3.screen.print(line)


# ---------------------------------------------------------
# TESTABLAUF
# ---------------------------------------------------------

# Gyro kalibriert sich beim Start -> Roboter nicht bewegen
show("Gyro kalibriert...", "nicht bewegen!")
wait(2000)
gyro.reset_angle(0)

ev3.speaker.beep(frequency=500, duration=200)
show("TEST START")
wait(1000)

# TEST 1: Geradeaus fahren (30 cm vor, Gyro-korrigiert)
show("1. Vorwaerts")
forward_result = gyro_straight(300)
show("1. Vorwaerts", "Abweichung: %d" % forward_result)
wait(1500)

# TEST 2: Rueckwaerts fahren (30 cm zurueck, Gyro-korrigiert)
show("2. Rueckwaerts")
backward_result = gyro_straight(-300)
show("2. Rueckwaerts", "Abweichung: %d" % backward_result)
wait(1500)

# TEST 3: Drehung nach links (90 Grad, Gyro)
show("3. Links 90")
left_result = gyro_turn(-90)
show("3. Links 90", "Gyro: %d" % left_result)
wait(1500)

# TEST 4: Drehung nach rechts (90 Grad, Gyro)
show("4. Rechts 90")
right_result = gyro_turn(90)
show("4. Rechts 90", "Gyro: %d" % right_result)
wait(1500)

# Test Ende: Ergebnisse stehen lassen
show("TEST FERTIG!",
     "Vor:    %d" % forward_result,
     "Zurueck: %d" % backward_result,
     "Links:  %d" % left_result,
     "Rechts: %d" % right_result)
ev3.speaker.beep(frequency=1000, duration=500)
wait(10000)
