from pybricks.parameters import Button
from pybricks.tools import wait

LINE_HEIGHT = 18


def wait_released(ev3):
    while ev3.buttons.pressed():
        wait(10)


def choose(ev3, title, entries):
    """Shows a menu and returns the index of the chosen entry (up/down + enter)."""
    selected = 0
    while True:
        ev3.screen.clear()
        ev3.screen.draw_text(0, 0, title)
        for i, entry in enumerate(entries):
            marker = "> " if i == selected else "  "
            ev3.screen.draw_text(0, (i + 1) * LINE_HEIGHT + 4, marker + entry)

        # wait for release first, enter is still held down after starting or aborting
        wait_released(ev3)
        while not ev3.buttons.pressed():
            wait(10)
        pressed = ev3.buttons.pressed()

        if Button.UP in pressed:
            selected = (selected - 1) % len(entries)
        elif Button.DOWN in pressed:
            selected = (selected + 1) % len(entries)
        elif Button.CENTER in pressed:
            wait_released(ev3)
            return selected
