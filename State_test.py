from dronekit import *
import droneapi
#import gps
import socket
import time
import sys
from pymavlink import mavutil
import argparse

parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect', default='115200', help="vehicle connection target. Default '57600'")
args = parser.parse_args()
IRIS = connect('/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00', baud = 115200, rate=6)
mode='active'
while mode=='active':
    print IRIS.attitude
    print IRIS.velocity
    time.sleep(1)
