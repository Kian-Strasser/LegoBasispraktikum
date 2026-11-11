#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import ColorSensor, Motor
from pybricks.hubs import EV3Brick
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

ev3 = EV3Brick()



#################################################
# TEMPORARY: INIT ROBOT & ROBOT COMPONENTS
#################################################

left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
color_sensor = ColorSensor(Port.S2)

robot = DriveBase(
    left_motor=left_motor,
    right_motor=right_motor,
    wheel_diameter=31,  # for distance calculations (in mm)
    axle_track=200,     # for rotation calculations
)



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

# PARAMETER FOR DERIVATIVE CALCULATION
last_error = 0



#################################################
# START LINE FOLLOWER
#################################################

# Start the line follower (beep and display message)
ev3.speaker.beep(frequency=1000, duration=200)
ev3.screen.print("Linienfolger Start")
wait(1000)



# main loop for line following
while True:
    brightness = color_sensor.reflection()
    
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
    robot.drive(int(speed), int(turn_rate))
    
    wait(10)