#!/usr/bin/env pybricks-micropython
# This program requires LEGO EV3 MicroPython v2.0 or higher.
import menu
from robot import Robot, Abort, BandCrossed
from sections import line_follow, color_search, push_cylinder, bridge


class Section:
    # MicroPython has no Enum, so just constants.
    # The numbers are the order in the parcours.
    LINE_FOLLOW = 0
    PUSH_CYLINDER = 2
    BRIDGE = 3
    COLOR_SEARCH = 1


# new section: create a file in sections/ with a run(robot) function,
# then add it to Section above and here
ROUTINES = {
    Section.LINE_FOLLOW: ("Linienfolgen", line_follow.run),
    Section.PUSH_CYLINDER: ("Zylinder schieben", push_cylinder.run),
    Section.BRIDGE: ("Bruecke", bridge.run),
    Section.COLOR_SEARCH: ("Farbfeldsuche", color_search.run)
}

robot = Robot()
names = [ROUTINES[section][0] for section in sorted(ROUTINES)]

while True:
    choice = menu.choose(robot.ev3, "Abschnitt waehlen", names + ["Beenden"])
    if choice == len(names):
        break

    # section the robot is in right now, moves on when the band is crossed
    current = choice
    while current < len(names):
        name, run = ROUTINES[current]
        robot.show("Abschnitt %d:" % (current + 1), name, "", "Enter: Menue")
        robot.off_band = False
        try:
            run(robot)
        except BandCrossed:
            pass
        except Abort:
            break
        finally:
            robot.stop()
        robot.ev3.speaker.beep()
        current += 1

robot.show("Programm beendet")
