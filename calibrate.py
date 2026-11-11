#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import ColorSensor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.tools import wait

ev3 = EV3Brick()
color_sensor = ColorSensor(Port.S1)

ev3.screen.print("--- KALIBRIERUNG ---")
wait(1000)

while True:
    # Misst die reflektierte Lichtstärke (0 bis 100)
    value = color_sensor.reflection()

    # Display leeren und aktuellen Wert anzeigen
    ev3.screen.clear()
    ev3.screen.print("Reflektion:")
    ev3.screen.print(value)

    # Wert alle 200 ms aktualisieren
    wait(200)