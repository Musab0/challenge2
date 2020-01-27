#
# Config for (takeoff_full.py)
#

class ArduConfig:

    # maximum buffer to look for object in
    MAX_BUFF = 20
    # detected time before object is considrerd detected
    DETECT_BUFF = 5
    # sleep time for loop
    LOOP_SPEED = 0.1
    max_box = 55
    # safe altitude
    SAFE_ALT = 5.0
    # safe altitude
    SAFE_ALT_aft = 0.3
    # alttitude change
    ALT_CHANGE = 0.5
    # alttitude change
    ALT_CHANGE_aft = 0.1

    #Image width=416, Dead zone area = [ 138-278 ], Left Partition =[0 - 138]
    #Right Partitioin= [278 - 416]
    #Image height=416, Dead zone area = [ 138-278 ], UPPER Partition =[0 - 138]
    #LOWER Partitioin= [278 - 416]
    # what is considered the left side of the frame in pixels
    #Note: Real Sense Camera Frame  : 640x480  
    LEFT_PARTITION = 320-100
    # what is considered the right side of the frame in pixels
    RIGHT_PARTITION =320+100
    # what is considered the upper side of the frame in pixels
    UPPER_PARTITION = 240-100
    # what is considered the down side of the frame in pixels
    LOWER_PARTITION = 240+100

    # turn speed slow
    TURN_SLOW_SPEED = 5
    # turn speed fast
    TURN_FAST_SPEED = 5
    # foreward speed when object is centred
    FOREWARD_SPEED = 1
    # BACKWARD speed when object is centred
    BACKWARD_SPEED = -1
    # time spent moving foreward
    FOREWARD_DURATION = 1
    BACKWARD_DURATION = 1
    # foreward 1 , stop 0
    FOREWARD_BOOL = 0
    BACKWARD_BOOL = 0


