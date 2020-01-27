#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys

from std_srvs.srv import Trigger, TriggerRequest

# Set up option parsing to get connection string
import argparse
import math
import time
from my_config_full import ArduConfig as config

import dronekit_sitl
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from dronekit import (LocationGlobal, LocationGlobalRelative, VehicleMode,
                      connect)
from pymavlink import mavutil  # Needed for command message definitions
from std_msgs.msg import Float32, Float64, Int8, String

parser = argparse.ArgumentParser(
    description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', default='127.0.0.1:14551',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

#--------------------------------------------------------------------------------
#a client to read the service provided by arduino to check the push botton status 
def pb_client(): 
   #print("Waiting for /pb_status Arduino Service")
   rospy.wait_for_service('/pb_status')
   #print("/pb_status service detected")
   try:
      pb_service = rospy.ServiceProxy('/pb_status', Trigger)
      pb = TriggerRequest();
      res=pb_service(pb);
      return res.success
   except rospy.ServiceException, e:
      print ("Service call failed: {}".format(e))

#--------------------------------------------------------------------------------
#a client to read the service provided by arduino to toggle the epm 
# returns the new epm status
def epm_client(): 
   #print("Waiting for /toggle_epm Arduino Service")
   rospy.wait_for_service('/toggle_epm')
   #print("//toggle_epm service detected")
   try:
      epm_service = rospy.ServiceProxy('/pb_status', Trigger)
      epm = TriggerRequest();
      res=epm_service(epm);
      return res.success
   except rospy.ServiceException, e:
      print ("Service call failed: {}".format(e))

#--------------------------------------------------------------------------------

class ArduCopter:
    def __init__(self):
        self.detect = False
        self.detected = False
        self.connection_string = args.connect
        self.detected_object = None
        self.vehicle = connect(self.connection_string, wait_ready=True)
        self.obj_detect_sub = rospy.Subscriber(
            "/darknet_ros/bounding_boxes", BoundingBoxes, self.bb_callback)
        self.sitl = None
        self.detected_no = rospy.Subscriber(
            "/darknet_ros/found_object", Int8, self.update_detected)

        self.start()

    def bb_callback(self, msg):
        #print(f'bounding box found {msg.bounding_boxes[0].Class}')
        # update variable when drone is detected
        self.detect = True
        self.detected_object = msg.bounding_boxes

    # Start SITL if no connection string specified

    def start(self):
        rospy.init_node("offboard_node")
        if not self.connection_string:
            self.sitl = dronekit_sitl.start_default()
            self.connection_string = self.sitl.connection_string()

    # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % self.connection_string)
        self.arm_and_takeoff(10)

    def update_detected(self, msg):
        self.detected = msg.data

    def goto_position_target_global_int(aLocation):
 
        msg = vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
            0b0000111111111000, # type_mask (only speeds enabled)
            aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
            aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
            aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
            0, # X velocity in NED frame in m/s
            0, # Y velocity in NED frame in m/s
            0, # Z velocity in NED frame in m/s
            0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
            # send command to vehicle
        vehicle.send_mavlink(msg)

    def goto_position_target_local_ned(self, north, east, up):
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
            0b0000111111111000,  # type_mask only positions enabled
            north, east, -up,  # x, y, z positions or North, East, Down in the MAV_FRAME_BODY_NED frame
            0, 0, 0,  # x, y, z velocity in m/s  not used
            0, 0, 0,  # x, y, z acceleration
            0, 0)    # yaw, yaw_rate
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def go_up(self, up):
        self.goto_position_target_local_ned(0, 0, up)

    def go_down(self, down):
        self.goto_position_target_local_ned(0, 0, -down)

    def arm_and_takeoff(self, aTargetAltitude):

        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        current_alt = self.vehicle.location.global_relative_frame.alt
        print("current altitude: ", current_alt)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        time.sleep(1)

        # check the alt is between -0.5 m and 1 m if true continue if not disarm
        if current_alt <= -0.5:
            self.vehicle.armed = False
            print("vehicle disarmed")
        elif current_alt >= 1:
            self.vehicle.armed = False
            print("vehicle disarmed")
        else:
            self.vehicle.armed = True
            print("vehicle armed")

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        time.sleep(3)
        # Take off to target altitude
        self.vehicle.simple_takeoff(aTargetAltitude)

        while True:
            print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
            # Trigger just below target alt.
            if self.vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def condition_yaw(self, heading, yaw_rate, left_right, relative=True):
        # params (heading in degress, yaw rate in deg/s, -1 is left and 1 is right, relative to body)
        if relative:
            is_relative = 1  # yaw relative to direction of travel
        else:
            is_relative = 0  # yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            heading,    # param 1, yaw in degrees
            yaw_rate,          # param 2, yaw speed deg/s
            left_right,          # param 3, direction -1 ccw, 1 cw
            is_relative,  # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)

    def turn_left(self, magnitude):
        self.condition_yaw(magnitude, 0, -1)

    def turn_right(self, magnitude):
        self.condition_yaw(magnitude, 0, 1)

    def stop(self):
        self.condition_yaw(0, 0, 1)

    def get_location_metres(self, original_location, dNorth, dEast):

        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation = LocationGlobal(
                newlat, newlon, original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation = LocationGlobalRelative(
                newlat, newlon, original_location.alt)
        else:
            raise Exception("Invalid Location object passed")

        return targetlocation

    def get_distance_metres(self, aLocation1, aLocation2):

        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def get_bearing(self, aLocation1, aLocation2):

        off_x = aLocation2.lon - aLocation1.lon
        off_y = aLocation2.lat - aLocation1.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing

    def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0, 0,
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(1)

    def goto(self, dNorth, dEast):

        currentLocation = self.vehicle.location.global_relative_frame
        targetLocation = self.get_location_metres(
            currentLocation, dNorth, dEast)
        targetDistance = self.get_distance_metres(
            currentLocation, targetLocation)
        self.vehicle.simple_goto(targetLocation)

        # Stop action if we are no longer in guided mode.
        while self.vehicle.mode.name == "GUIDED":
            remainingDistance = self.get_distance_metres(
                self.vehicle.location.global_relative_frame, targetLocation)
            print("Distance to target: ", remainingDistance)
            # Just below target, in case of undershoot.
            if remainingDistance <= targetDistance*0.01:
                print("Reached target")
                break
            time.sleep(2)

    ################################################



       

    def chase(self):
        #
        # imported vatiables from config
        #

        # maximum buffer to look for object in
        MAX_BUFF = config.MAX_BUFF
        # detected time before object is considrerd detected
        DETECT_BUFF = config.DETECT_BUFF
        # sleep time for loop
        LOOP_SPEED = config.LOOP_SPEED
        # safe altitude
        SAFE_ALT = config.SAFE_ALT
        # safe altitude
        SAFE_ALT_aft = config.SAFE_ALT_aft
        # alttitude change
        ALT_CHANGE = config.ALT_CHANGE
        # alttitude change
        ALT_CHANGE_aft = config.ALT_CHANGE_aft
        # what is considered the left side of the frame in pixels
        LEFT_PARTITION = config.LEFT_PARTITION
        # what is considered the right side of the frame in pixels
        RIGHT_PARTITION = config.RIGHT_PARTITION
        # what is considered the upper side of the frame in pixels
        UPPER_PARTITION = config.UPPER_PARTITION
        # what is considered the lower side of the frame in pixels
        LOWER_PARTITION = config.LOWER_PARTITION
        # turn speed slow
        TURN_SLOW_SPEED = config.TURN_SLOW_SPEED
        # turn speed fast
        TURN_FAST_SPEED = config.TURN_FAST_SPEED
        # foreward speed when object is centred
        FOREWARD_SPEED = config.FOREWARD_SPEED
        # BACKWARD speed when object is centred
        BACKWARD_SPEED = config.BACKWARD_SPEED
        # time spent moving foreward
        FOREWARD_DURATION = config.FOREWARD_DURATION
        # time spent moving BACKWARD
        BACKWARD_DURATION = config.BACKWARD_DURATION
        # move foreward or stop when detected in middle
        FOREWARD_BOOl = config.FOREWARD_BOOL

        # flag triggered when an object is on the left side
        left_flag = False
        # flag triggered when an object is on the right side
        right_flag = False
        # flag triggered when an object is on the upper side
        upper_flag = False
        # flag triggered when an object is on the lower side
        lower_flag = False

        # flag that is updated when something is detected
        detected_flag = False

        # var decliration
        buffer = 0	
        detected_s = 0
        print('intiated')
        descend_flag = False
        up_flag = False
        epm_state = epm_client()

        while 1:
            
	   # print('while')
            #pb=pb_client()
            
            #
            # Buffer Detection
            #
            buffer = buffer + 1
            
            if self.detected > 0:
                detected_s = detected_s + 1

            if detected_s >= DETECT_BUFF and buffer == MAX_BUFF:
          #      print("detected")
                detected_flag = True

            elif detected_s < DETECT_BUFF and buffer == MAX_BUFF:
          #      print("not detected")
                detected_flag = False

            if buffer == MAX_BUFF:
                buffer = 0
                detected_s = 0

            
           
            try:
              #  print('trying')
              #  print('push button:                       {}'.format(pb_client()))
                up_flag = pb_client()
                x = self.detected_object[0]
                detected_class = x.Class
                ymin=x.ymin
                ymax=x.ymax
                xmin = x.xmin
                xmax = x.xmax
                center_flag = False
                align_flag = False
                C_A_flag = False

              #  print('box middle is {} and the class is {}'.format(
              #     (xmin+xmax)/2, detected_class))

                # roll left when an object is detected in LEFT_PARTITION
                if (xmin+xmax)/2 < LEFT_PARTITION and detected_flag is True:
                    print("roll left")
                    left_flag = True
                    right_flag = False
                    self.send_ned_velocity(0, BACKWARD_SPEED,0, BACKWARD_DURATION)

                # roll right
                elif (xmin+xmax)/2 > RIGHT_PARTITION and detected_flag is True:
                    right_flag = True
                    left_flag = False
                    print("roll right")
                    self.send_ned_velocity(0, FOREWARD_SPEED,0,FOREWARD_DURATION)

                # pitch forward
                elif (ymin+ymax)/2 < UPPER_PARTITION and detected_flag is True:
                    print("pitch forward")
                    upper_flag = True
                    lower_flag = False
                    self.send_ned_velocity(FOREWARD_SPEED,0,0,FOREWARD_DURATION)

                # pitch backward
                elif (ymin+ymax)/2 > LOWER_PARTITION and detected_flag is True:
                    lower_flag = True
                    upper_flag = False
                    print("Pitch backward")
                    self.send_ned_velocity(BACKWARD_SPEED,0,0,BACKWARD_DURATION)

                elif detected_flag is True:
                    center_flag=True
                    print("Brick Centered !")

                elif detected_flag is False and left_flag is False and right_flag is False and upper_flag is False and lower_flag is False:
                    print("no target found, searching...")
                    self.stop()

                if center_flag is True and align_flag is False:
                    ratio=((1.0*xmax-xmin)/(1.0*ymax-ymin))*100.0
                    print('x/y*100 ratio                             {}'.format(ratio))
                    if (ratio >= 135 and ratio <150) or (ratio >= 197 and ratio <205): 
                        self.stop()
                        align_flag=True
                        print("Brick Aligned !")
                    else:
                        self.turn_left(TURN_SLOW_SPEED)
                        print("Yaw left")
                        time.sleep(LOOP_SPEED)
                if center_flag is True and align_flag is True:
                    C_A_flag = True
                else:
                    C_A_flag = False 
  

                if descend_flag is False and C_A_flag is True:
                    self.go_down(SAFE_ALT) 
                    while 1 : 
                       if not (self.vehicle.location.global_relative_frame.alt >= SAFE_ALT+0.2):
                         break

                    print('The brick descended 5m')   
                    print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
                    descend_flag = True  
 
                if descend_flag is True and pb_client() is False and (self.vehicle.location.global_relative_frame.alt - 0.2) <= SAFE_ALT and (self.vehicle.location.global_relative_frame.alt >=0.1) and C_A_flag is True:
                    new_pos=self.vehicle.location.global_relative_frame.alt - 0.5
                    self.go_down(0.5) 
                    while 1 : 
                       if not (self.vehicle.location.global_relative_frame.alt >= new_pos+0.1):
                         break
                       if pb_client() is True:
                         break
#                     print('The brick descended 0.5m')
#                     print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)

                elif descend_flag is True and pb_client() is True:
                        self.go_up(10)
                        print ('go up')
                        print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
                        break


            except:
                print("exception...")
                print(" e-Altitude: ", self.vehicle.location.global_relative_frame.alt)
            time.sleep(LOOP_SPEED)



    def main(self):
        self.goto(0,0)
        self.chase()


        # sleep period
        time.sleep(0.5)

        print("Setting LAND mode...")
        self.vehicle.mode = VehicleMode("LAND")

        # Close vehicle object before exiting script
        print("Close vehicle object")
        self.vehicle.close()

        # Shut down simulator if it was started.
        if self.sitl is not None:
            self.sitl.stop()

        print("Completed")


if __name__ == "__main__":
    con = ArduCopter()
    con.main()
