from dronekit import *
import droneapi
import numpy as np
#import gps
import socket
import time
import sys
from pymavlink import mavutil
import argparse


def Connect2SOLO():
    print("Attempting Connection to SOLO")
    parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
    parser.add_argument('--connect', default='115200', help="vehicle connection target. Default '57600'")
    args = parser.parse_args()
    SOLO = connect('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00', baud = 115200, rate=6)
    if SOLO.armed == True:
        print "SOLO has Successfully Connected!"
    else:
        print "SOLO has not connected yet..."    
    return SOLO

def SetGuidedMode():
    print "Setting SOLO to Guided Mode. Be sure sure controller is in 'AUTO' Mode"
    print "Current Mode: %s" % SOLO.mode.name
    time.sleep(1)
    SOLO.mode = VehicleMode("GUIDED")
    while not SOLO.mode.name == 'GUIDED':
        print "Waiting for SOLO mode change..."
        print "Current Mode: %s" % UAVS.mode.name
        time.sleep(1)

def ArmSOLO():
    UAVS.armed = True
    while not UAVS.armed:
        print "UAVS waiting to arm..."
        time.sleep(1)
    #if UAVS.armed = True:
        #print " Vehcile is armed: %s" % UAVS.armed
        
def Takeoff(aTargetAltitude, SOLO):
    #if UAVS.Armed = True:
    print" TAKING OFF - STAND CLEAR!"
    UAVS.simple_takeoff(aTargetAltitude)
    while True:
        print " Altitude: ", UAVS.location.global_relative_frame.alt
        if UAVS.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print "Arrived at Target Altitude"
            break

def send_NED_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = UAVS.message_factory.set_position_target_local_ned_encode(
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
        UAVS.send_mavlink(msg)
        time.sleep(1)    

def SetLandMode():
    print "Setting SOLO to LAND Mode..."
    UAVS.mode = VehicleMode("LAND")
    print "Current Mode: %s" % UAVS.mode.name


#/////////////////////////MISSION SCRIPTS\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

def Maneuver_1(aTargetAltitude):
    print "Starting Basic Pre-Arm Checks..."
    print UAVS.mode.name
    if UAVS.mode.name == "INITIALISING":
        print "Waiting for UAV-S to initalise"
        time.sleep(1)
    print UAVS.gps_0.fix_type
    while UAVS.gps_0.fix_type < 2:
        print "Waiting for GPS...", UAVS.gps_0.fix_type
        time.sleep(1)
    print "Setting UAV-S to Guided Mode..."

    print "Current Mode: %s" % UAVS.mode.name
    time.sleep(1)
    #while not UAVS.is_armable:
        #print("Waiting for UAV-S to initialise...")
        #time.sleep(1)

    print"Arming Motors - STAND CLEAR"
    UAVS.mode = VehicleMode("GUIDED")
    UAVS.armed = True
    UAVS.flush()

    while not UAVS.armed:
        print "Waiting for SOLO mode change..."
        print "Current Mode: %s" % UAVS.mode.name
        time.sleep(1)           

    print" TAKING OFF - STAND CLEAR!"
    time.sleep(4)
    UAVS.simple_takeoff(aTargetAltitude)
    #UAVS.commands.takeoff(aTargetAltitude) # Takeoff to target alttude
    UAVS.flush()
    
    while True:
        print " Altitude: ", UAVS.location.global_relative_frame.alt
        if UAVS.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print "Arrived at Target Altitude"
            break
        time.sleep(1)
    time.sleep(60)

    print "Setting SOLO to LAND Mode"
    UAVS.mode = VehicleMode("LAND")
    while True:
        print " Altitude: ", UAVS.location.global_relative_frame.alt
        if UAVS.location.global_relative_frame.alt <= 0:
            print "ARRIVED AT GROUND"
            UAVS.armed = False
            break
        time.sleep(1)
    # print"Current Battery Level: %s" % UAVS.battery.level
    print "CONGRATULATIONS - MISSION ACOMPLISHED"
    print "Closing SOLO object"
    UAVS.close()
    UAVS.flush()   

#/////////////////////////START OF SCRIPT\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

print("Attempting Connection to UAV-S")
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', default='115200', help="vehicle connection target. Default '57600'")
args = parser.parse_args()
UAVS = connect('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00', baud = 115200, rate=6)

#Maneuver_1(15)
   
#c=np.concatenate((UAVS.location.local_frame,UAVS.velocity,UAVS.attitude),axis=0) 
if __name__=="__main__":
    d=str(UAVS.location.local_frame)
    v=str(UAVS.velocity)
    #puts velocity values into an array
    last_comma_index=0
    num_vel=0
    velocity=np.zeros((3,1))
    for i in range(0,len(v)):
        if v[i]==',':
            velocity[num_vel]=float(v[(last_comma_index+1):i])
            last_comma_index=i
            num_vel+=1
        if v[i] == ']':
            velocity[2]=float(v[last_comma_index+1:i])

    #puts attitude values into an array
    a=str(UAVS.attitude)
    last_comma_index=0
    last_eq_index=0
    num_vel=0
    attitude=np.zeros((3,1))
    for i in range(0,len(a)):
        if a[i]=='=':
            last_eq_index=i
        if a[i]==',':
            attitude[num_vel]=float(a[last_eq_index+1:i])
            num_vel+=1
        if i==len(a)-1:
            attitude[2]=float(a[last_eq_index+1:i])
            
    print a
    print attitude

















    
