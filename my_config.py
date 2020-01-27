#
# Config for (takeoff_yaw_body.py)
#

class ArduConfig:

    # maximum buffer to look for object in
    MAX_BUFF = 20
    # detected time before object is considrerd detected
    DETECT_BUFF = 5

    max_box = 55

    # what is considered the left side of the frame in pixels
    LEFT_PARTITION = 250
    # what is considered the right side of the frame in pixels
    RIGHT_PARTITION = 450

    # turn speed slow
    TURN_SLOW_SPEED = 5
    # turn speed fast
    TURN_FAST_SPEED = 5
    # foreward 1 , stop 0
    FOREWARD_BOOL = 0
    # foreward speed when object is centred
    FOREWARD_SPEED = 1
    # time spent moving foreward
    FOREWARD_DURATION = 2
    #

    # sleep time for loop
    LOOP_SPEED = 0.1
