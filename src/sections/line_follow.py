from pybricks.tools import StopWatch

from robot import Robot

#################################################
# LINE FOLLOWER PARAMETERS
#################################################

# PARAMETER FOR LIGHT SENSOR
# measured 2026-11-18 with wheels (tracks: black 11, white 76)
BLACK_VALUE = 10
WHITE_VALUE = 65
THRESHOLD = (BLACK_VALUE + WHITE_VALUE) / 2

# PARAMETER FOR PID CONTROLLER
KP = 1.2 # P-Coeffizient increased: faster response to deviations
KD = 0.8 # D-Coeffizient increased: faster response to changes in deviation
# second stage: far off the edge (almost all white or all board) it turns
# harder, KP alone was too weak for a sharp right curve at 80 mm/s and the
# sensor slid across the whole line. near the edge nothing changes
KP_FAR_START = 12  # |error| from which the extra turn starts (was 15)
KP_FAR = 5         # extra deg/s per unit of error above that (was 3, not
                   # enough for the sharp right curve at 100 mm/s)

# PARAMETERS FOR SPEED CONTROL (in mm/s)
BASE_SPEED = 100 # base speed for straight sections (was 60, then 80)
MIN_SPEED = 20  # minimum speed to prevent stalling in sharp turns (was 15)
# no slowing down for small errors, then SLOW_DOWN mm/s less per unit of
# error, so it still gets down to MIN_SPEED at the largest error (27)
# (was 2.5 per unit from 0 on, that braked a lot on straights)
SLOW_DOWN_START = 8
SLOW_DOWN = 4.2

# the band check switches the sensor mode away from reflection() and back,
# that costs about 70 ms. the band (2-3 cm) is dark in reflection(), so it is
# checked whenever the sensor gets dark, the timed check is only a fallback
# (250 alone could drive over the band between two checks)
BAND_INTERVAL = 1000


#################################################
# GAP PARAMETERS
#################################################

# the follower rides the right edge of the white line: when the line is gone
# it is either a left corner (line is found by turning left) or a gap

DARK_VALUE = 20    # reflection below this = only the board, no line
LOST_TIME = 300    # ms of only board until the line counts as lost
HEADING_SMOOTHING = 0.05  # share of the new gyro value in the average heading

LOST_ADVANCE = 15   # mm straight on past the end of the line before turning
                    # (30 with tracks, the sensor sits further forward now)
CORNER_ANGLE = 110  # degrees to turn left looking for a corner before it is a gap
                    # (90 missed a sharp left curve just past the limit)
SEARCH_RATE = 60    # deg/s while looking for the line (was 45)
TURN_RATE = 60      # deg/s when turning back (was 45)
TURN_SLOW_RATE = 15  # deg/s for the last 20 degrees
TURN_LEAD = 1       # degrees, stop this early, the robot turns on after braking
                    # (must stay below TURN_TOLERANCE, or a correction can stall)
TURN_TOLERANCE = 2  # degrees, done if this close to the target
TURN_SETTLE = 120   # ms after braking until the gyro value is settled

# a loose gyro cable freezes the angle and the robot spins forever,
# so a turn stops if the angle moves less than this in that time
GYRO_MIN_CHANGE = 5  # degrees, the gyro jitters by about 3 when standing
GYRO_TIMEOUT = 2000  # ms

GAP_MAX = 150       # mm to drive straight over a gap (gaps are 100 at most)
GAP_SPEED = 80      # mm/s over a gap (was 60)
GAP_KP = 3          # gyro correction over a gap, same as Robot.straight()
GAP_BIAS = 8        # degrees to the left of the heading, so the sensor runs
                    # onto the line after the gap and not along its edge;
                    # behind some gaps the line is 2-3 cm further left. only
                    # works because the follower rides the right edge: it
                    # crosses the line from right to left, flip it for the left edge
SWEEP_ANGLES = (30, 60)  # degrees left and right if the line is still gone


def on_line(robot):
    return robot.color_sensor.reflection() > THRESHOLD


def search(robot, target):
    """Turns on the spot to the gyro angle target, True as soon as the line is seen."""
    direction = 1 if target > robot.gyro.angle() else -1
    watch = GyroWatch(robot)
    while (target - robot.gyro.angle()) * direction > 0:
        if on_line(robot):
            return True
        robot.drive_base.drive(0, direction * SEARCH_RATE)
        robot.wait(10)
        watch.check()
    return False


class GyroWatch:
    """Stops the robot when the gyro angle stops changing during a turn."""

    def __init__(self, robot):
        self.robot = robot
        self.angle = robot.gyro.angle()
        self.timer = StopWatch()

    def check(self):
        angle = self.robot.gyro.angle()
        if abs(angle - self.angle) >= GYRO_MIN_CHANGE:
            self.angle = angle
            self.timer.reset()
        elif self.timer.time() > GYRO_TIMEOUT:
            self.robot.brake()
            self.robot.show("Gyro Fehler", "Kabel S1 pruefen", "", "Enter: Menue")
            self.robot.ev3.speaker.beep(frequency=300, duration=500)
            while True:
                self.robot.wait(100)


def turn_to(robot, target):
    """Turns on the spot to the gyro angle target without looking for the line.

    The gyro value is a bit late and the robot turns too far, so the last
    degrees are slow, it stops a bit early and corrects until it is within
    TURN_TOLERANCE.
    """
    for _ in range(5):
        error = target - robot.gyro.angle()
        if abs(error) <= TURN_TOLERANCE:
            break
        direction = 1 if error > 0 else -1
        watch = GyroWatch(robot)
        while (target - robot.gyro.angle()) * direction > TURN_LEAD:
            watch.check()
            rate = TURN_RATE if abs(target - robot.gyro.angle()) > 20 else TURN_SLOW_RATE
            robot.drive_base.drive(0, direction * rate)
            robot.wait(10)
        robot.brake()
        robot.wait(TURN_SETTLE)


def straight_to_line(robot, target, distance):
    """Drives straight on the gyro angle target, True as soon as the line is seen.

    False if there was no line within distance mm.
    """
    robot.drive_base.stop()
    robot.drive_base.reset()
    while robot.drive_base.distance() < distance:
        if on_line(robot):
            return True
        correction = GAP_KP * (target - robot.gyro.angle())
        robot.drive_base.drive(GAP_SPEED, correction)
        robot.wait(10)
    return False


def find_line(robot, heading):
    """Called when the line is lost, returns when the sensor is on it again.

    heading is the gyro angle the robot had while it was still following.
    """
    # first a bit further straight on: right behind the end of the line the
    # sensor finds the line the robot is still standing on when it turns
    turn_to(robot, heading)
    if straight_to_line(robot, heading, LOST_ADVANCE):
        return

    # left corner: the line is found by turning left
    if search(robot, heading - CORNER_ANGLE):
        return

    # gap: back to the old heading and straight on until the line is there
    target = heading - GAP_BIAS
    turn_to(robot, target)
    if straight_to_line(robot, target, GAP_MAX):
        return

    # still nothing: sweep left and right, wider each time
    for angle in SWEEP_ANGLES:
        if search(robot, heading - angle) or search(robot, heading + angle):
            return

    robot.brake()
    robot.show("Linie verloren", "", "", "Enter: Menue")
    robot.ev3.speaker.beep(frequency=300, duration=500)
    while True:
        robot.wait(100)


def run(robot: Robot):
    # PARAMETER FOR DERIVATIVE CALCULATION
    last_error = 0

    robot.band_interval = BAND_INTERVAL

    # average gyro angle while following, the follower always swings a bit
    heading = robot.gyro.angle()
    dark_timer = StopWatch()
    was_bright = True

    # Start the line follower (beep, the section name is already on the display)
    robot.ev3.speaker.beep(frequency=1000, duration=200)
    robot.wait(1000)

    # main loop for line following
    while True:
        brightness = robot.color_sensor.reflection()

        # gaps and lost line, the controller below stays as it is
        if brightness >= DARK_VALUE:
            dark_timer.reset()
            heading += HEADING_SMOOTHING * (robot.gyro.angle() - heading)
        elif dark_timer.time() > LOST_TIME:
            find_line(robot, heading)
            heading = robot.gyro.angle()
            dark_timer.reset()
            was_bright = True
            continue

        # just got dark: the board or the band
        if brightness < DARK_VALUE and was_bright:
            robot.check_band()
        was_bright = brightness >= DARK_VALUE

        # 1. Calculate error
        error = brightness - THRESHOLD

        # 2. Calculate derivative
        derivative = error - last_error
        last_error = error

        # 3. Calculate turn rate
        turn_rate = (error * KP) + (derivative * KD)
        if abs(error) > KP_FAR_START:
            far = (abs(error) - KP_FAR_START) * KP_FAR
            turn_rate += far if error > 0 else -far

        #4. Dynamic curve break
        # Bremst bei Abweichung scharf ab, damit der Roboter nicht aus der Bahn fliegt
        slow_down = max(0, abs(error) - SLOW_DOWN_START) * SLOW_DOWN
        speed = max(MIN_SPEED, BASE_SPEED - slow_down)

        #5. Drive the robot
        robot.drive_base.drive(int(speed), int(turn_rate))

        robot.wait(10)
