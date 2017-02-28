from dronekit import *
import droneapi
#import gps
import socket
import time
import sys
from pymavlink import mavutil
import argparse


def Connect2IRIS():
    print("Attempting Connection to IRIS")
    parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
    parser.add_argument('--connect', default='115200', help="vehicle connection target. Default '57600'")
    args = parser.parse_args()
    IRIS = connect('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00', baud = 115200, rate=6)
    if IRIS.armed == True:
        print "IRIS has Successfully Connected!"
    else:
        print "IRIS has not connected yet..."    
    return IRIS

def SetGuidedMode():
    print "Setting IRIS to Guided Mode. Be sure sure controller is in 'AUTO' Mode"
    print "Current Mode: %s" % IRIS.mode.name
    time.sleep(1)
    IRIS.mode = VehicleMode("GUIDED")
    while not IRIS.mode.name == 'GUIDED':
        print "Waiting for IRIS mode change..."
        print "Current Mode: %s" % IRIS.mode.name
        time.sleep(1)

def ArmIRIS():
    IRIS.armed = True
    while not IRIS.armed:
        print "IRIS waiting to arm..."
        time.sleep(1)
    #if IRIS.armed = True:
        #print " Vehcile is armed: %s" % IRIS.armed
        
def Takeoff(aTargetAltitude, IRIS):
    #if IRIS.Armed = True:
    print" TAKING OFF - STAND CLEAR!"
    IRIS.simple_takeoff(aTargetAltitude)
    while True:
        print " Altitude: ", IRIS.location.global_relative_frame.alt
        if IRIS.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print "Arrived at Target Altitude"
            break

def send_NED_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = IRIS.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x-, y-, z-positions (not used)
        velocity_x, velocity_y, velocity_z, # x-, y-, z-velocity in m/s
        0, 0, 0, # x-,y,-z-acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    #yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)    
    # send command to vehicle on 1Hz cycle
    for x in range(0, duration):
        IRIS.send_mavlink(msg)
        time.sleep(1)    

def SetLandMode():
    print "Setting IRIS to LAND Mode..."
    IRIS.mode = VehicleMode("LAND")
    print "Current Mode: %s" % IRIS.mode.name


#/////////////////////////MISSION SCRIPTS\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

def Maneuver_1(aTargetAltitude):
    print "Starting Basic Pre-Arm Checks..."

    print "Setting IRIS to Guided Mode. Be sure sure controller is in 'AUTO' Mode"

    print "Current Mode: %s" % IRIS.mode.name
    time.sleep(1)
    #while not IRIS.is_armable:
        #print("Waiting for IRIS to initialise...")
        #time.sleep(1)

    print"Arming Motors - STAND CLEAR"
    IRIS.mode = VehicleMode("GUIDED")
    time.sleep(4)
    IRIS.armed = True
    while not IRIS.armed:
        print "Waiting for IRIS mode change..."
        print "Current Mode: %s" % IRIS.mode.name
        IRIS.armed = True
        time.sleep(1)           

    print" TAKING OFF - STAND CLEAR!"
    time.sleep(4)
    IRIS.simple_takeoff(aTargetAltitude)
    while True:
        print " Altitude: ", IRIS.location.global_relative_frame.alt
        if IRIS.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print "Arrived at Target Altitude"
            break
        time.sleep(1)
    time.sleep(5)

    print "Setting IRIS to LAND Mode"
    IRIS.mode = VehicleMode("LAND")
    while True:
        print " Altitude: ", IRIS.location.global_relative_frame.alt
        if IRIS.location.global_relative_frame.alt <= 0:
            print "ARRIVED AT GROUND"
            IRIS.armed = False
            break
        time.sleep(1)
    # print"Current Battery Level: %s" % IRIS.battery.level
    print "CONGRATULATIONS - MISSION ACOMPLISHED"
    print "Closing IRIS object"
    IRIS.close()
    print "Closing IRIS object"
    IRIS.close()    

def Maneuver_2(aTargetAltitude):
    print "Starting Basic Pre-Arm Checks..."

    print "Setting IRIS to Guided Mode. Be sure sure controller is in 'AUTO' Mode"

    print "Current Mode: %s" % IRIS.mode.name
    time.sleep(1)
    #while not IRIS.is_armable:
        #print("Waiting for IRIS to initialise...")
        #time.sleep(1)

    print"Arming Motors - STAND CLEAR"
    IRIS.mode = VehicleMode("GUIDED")
    time.sleep(4)
    IRIS.armed = True
    while not IRIS.armed:
        print "Waiting for IRIS mode change..."
        print "Current Mode: %s" % IRIS.mode.name
        IRIS.armed = True
        time.sleep(1)           

    print" TAKING OFF - STAND CLEAR!"
    time.sleep(4)
    IRIS.simple_takeoff(aTargetAltitude)
    while True:
        print " Altitude: ", IRIS.location.global_relative_frame.alt
        if IRIS.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print "Arrived at Target Altitude"
            break
        time.sleep(1)
    time.sleep(5)
    print "Velocity Vector sent: vx=2, vy=vz=0 for t=6 seconds"
    send_NED_velocity(1, 0, 0, 5)
    print "Velocity Vector sent: vx=vy=vz=0 for t=2 seconds"
    send_NED_velocity(0, 0, 0, 2)
    time.sleep(5)

    print "Setting IRIS to LAND Mode"
    IRIS.mode = VehicleMode("LAND")
    while True:
        print " Altitude: ", IRIS.location.global_relative_frame.alt
        if IRIS.location.global_relative_frame.alt <= 0:
            print "ARRIVED AT GROUND"
            IRIS.armed = False
            break
        time.sleep(1)
    print"Current Battery Level: %s" % IRIS.battery.level
    print "CONGRATULATIONS - MISSION ACOMPLISHED"
    print "Closing IRIS object"
    IRIS.close()
#/////////////////////////START OF SCRIPT\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

print("Attempting Connection to IRIS")
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', default='115200', help="vehicle connection target. Default '57600'")
args = parser.parse_args()
IRIS = connect('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00', baud = 115200, rate=6)
#print"Current Battery Level: %s" % IRIS.battery.level

Maneuver_1(3)
   



    
