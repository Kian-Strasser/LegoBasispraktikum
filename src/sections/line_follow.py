from robot import Robot

#################################################
# LINE FOLLOWER PARAMETERS
#################################################

# PARAMETER FOR LIGHT SENSOR
BLACK_VALUE = 11
WHITE_VALUE = 76
THRESHOLD = (BLACK_VALUE + WHITE_VALUE) / 2

# PARAMETER FOR PID CONTROLLER
KP = 1.2 # P-Coeffizient increased: faster response to deviations
KD = 0.8 # D-Coeffizient increased: faster response to changes in deviation

# PARAMETERS FOR SPEED CONTROL (in mm/s)
BASE_SPEED = 60 # base speed for straight sections
MIN_SPEED = 15  # minimum speed to prevent stalling in sharp turns

# the band check switches the sensor mode away from reflection() and back,
# that costs about 70 ms, so only do it every BAND_INTERVAL ms
BAND_INTERVAL = 250


def run(robot: Robot):
    # PARAMETER FOR DERIVATIVE CALCULATION
    last_error = 0

    robot.band_interval = BAND_INTERVAL

    # Start the line follower (beep, the section name is already on the display)
    robot.ev3.speaker.beep(frequency=1000, duration=200)
    robot.wait(1000)

    # main loop for line following
    while True:
        brightness = robot.color_sensor.reflection()

        # 1. Calculate error
        error = brightness - THRESHOLD

        # 2. Calculate derivative
        derivative = error - last_error
        last_error = error

        # 3. Calculate turn rate
        turn_rate = (error * KP) + (derivative * KD)

        #4. Dynamic curve break
        # Bremst bei Abweichung scharf ab, damit der Roboter nicht aus der Bahn fliegt
        speed = max(MIN_SPEED, BASE_SPEED - (abs(error) * 2.5))

        #5. Drive the robot
        robot.drive_base.drive(int(speed), int(turn_rate))

        robot.wait(10)
