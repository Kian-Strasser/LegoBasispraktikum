from robot import Robot


def run(robot: Robot):
    # TODO state machine from sheet 2 exercise 2
    # for now it just waits for enter or the band
    while True:
        robot.wait(100)
