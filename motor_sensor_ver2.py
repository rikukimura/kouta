#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Author: Sun Haonan

# *********     Sync Read and Sync Write Example      *********

################################################################################

import os, time
from struct import *
import socket
from datetime import datetime
import getopt, glob
sample = 10 #written by kato 


if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                     # Uses Dynamixel SDK library

def tic():
    #Homemade version of matlab tic and toc functions
    import time
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()

def toc():
    import time
    if 'startTime_for_tictoc' in globals():
        print('Elapsed time is ' + str(time.time() - startTime_for_tictoc) + ' seconds.')
    else:
        print('Toc: start time not set')


# Control table address
ADDR_MX_HOMING_OFFSET           = 20            # Control table address is different in Dynamixel model
ADDR_MX_TORQUE_ENABLE           = 64
ADDR_MX_LED                     = 65
ADDR_MX_PROFILE_ACCELERATION    = 108
ADDR_MX_PROFILE_VELOCITY        = 112
ADDR_MX_GOAL_POSITION           = 116
ADDR_MX_PRESENT_CURRENT         = 126
ADDR_MX_PRESENT_VELOCITY        = 128
ADDR_MX_PRESENT_POSITION        = 132

# Data Byte Length
LEN_MX_GOAL_POSITION            = 4
LEN_MX_PRESENT_POSITION         = 4
LEN_MX_PRESENT_CURRENT          = 2
LEN_MX_PRESENT_VELOCITY         = 4
LEN_MX_PROFILE_VELOCITY         = 4

# Protocol ve+rsion
PROTOCOL_VERSION                = 2.0           # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                         = 1             # Dynamixel#1 ID : 1
DXL2_ID                         = 2             # Dynamixel#1 ID : 2
DXL3_ID                         = 3             # Dynamixel#1 ID : 3
BAUDRATE                        = 1000000       # Dynamixel default baudrate : 57600
DEVICENAME                      = 'COM7'        # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE                   = 1             # Value for enabling the torque
TORQUE_DISABLE                  = 0             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE      = 100           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE      = 4000          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD     = 1             # Dynamixel moving status threshold

ESC_ASCII_VALUE                 = 0x1b
COMM_SUCCESS                    = 0             # Communication Success result value
COMM_TX_FAIL                    = -1001         # Communication Tx Failed

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite1 = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
groupSyncWrite2 = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_PROFILE_VELOCITY, LEN_MX_PROFILE_VELOCITY)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)

# Multi Turn Offset
offset_W = 3938
offset_S = 985
offset_C = 47

# Angular speed of the wheel
w = 10
dxl1_profile_velocity = 40
dxl2_profile_velocity = 40
dxl3_profile_velocity = 40

# Initial angular position of the wheel, the sun gear, and the carrier (unit:degree)
ini_W_angle = 0       # Wheel
ini_S_angle = 0       # Sun gear
ini_C_angle = 0       # Carrier

ini_W_position = int(ini_W_angle / 360 * 4096 + offset_W)          # Wheel
ini_C_position = int(ini_C_angle / 360 * 4096 + offset_C)          # Carrier
ini_S_position = int(ini_S_angle / 360 * 4096 + offset_S)          # Sun gear

dxl_comm_result = COMM_TX_FAIL             # Communication result
dxl_error = 0                              # Dynamixel error

# Goal position
dxl1_goal_position = ini_W_position
dxl2_goal_position = ini_C_position
dxl3_goal_position = ini_S_position

# Present position
dxl1_present_position = 0
dxl2_present_position = 0
dxl3_present_position = 0

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)

# Enable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL3_ID)

# Add parameter storage for Dynamixel#1 #2 #3 present position value
dxl_addparam_result = groupSyncRead.addParam(DXL1_ID)
dxl_addparam_result = groupSyncRead.addParam(DXL2_ID)
dxl_addparam_result = groupSyncRead.addParam(DXL3_ID)

# Allocate goal position value into byte array
param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl1_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl1_goal_position))]
param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position))]
param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(dxl3_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl3_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl3_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl3_goal_position))]

# Initialization
# Add Dynamixel#1 #2 #3 goal position value to the Syncwrite parameter storage
dxl_addparam_result = groupSyncWrite1.addParam(DXL1_ID, param_goal_position1)
dxl_addparam_result = groupSyncWrite1.addParam(DXL2_ID, param_goal_position2)
dxl_addparam_result = groupSyncWrite1.addParam(DXL3_ID, param_goal_position3)

# Syncwrite goal position
dxl_comm_result = groupSyncWrite1.txPacket()

# Clear syncwrite parameter storage
groupSyncWrite1.clearParam()

# Allocate profile velocity value into byte array
param_profile_velocity1 = [DXL_LOBYTE(DXL_LOWORD(dxl1_profile_velocity)), DXL_HIBYTE(DXL_LOWORD(dxl1_profile_velocity)), DXL_LOBYTE(DXL_HIWORD(dxl1_profile_velocity)), DXL_HIBYTE(DXL_HIWORD(dxl1_profile_velocity))]
param_profile_velocity2 = [DXL_LOBYTE(DXL_LOWORD(dxl2_profile_velocity)), DXL_HIBYTE(DXL_LOWORD(dxl2_profile_velocity)), DXL_LOBYTE(DXL_HIWORD(dxl2_profile_velocity)), DXL_HIBYTE(DXL_HIWORD(dxl2_profile_velocity))]
param_profile_velocity3 = [DXL_LOBYTE(DXL_LOWORD(dxl3_profile_velocity)), DXL_HIBYTE(DXL_LOWORD(dxl3_profile_velocity)), DXL_LOBYTE(DXL_HIWORD(dxl3_profile_velocity)), DXL_HIBYTE(DXL_HIWORD(dxl3_profile_velocity))]

# Add Dynamixel#1 #2 #3 profile velicity value to the Syncwrite storage
dxl_addparam_result = groupSyncWrite2.addParam(DXL1_ID, param_profile_velocity1)
dxl_addparam_result = groupSyncWrite2.addParam(DXL2_ID, param_profile_velocity2)
dxl_addparam_result = groupSyncWrite2.addParam(DXL3_ID, param_profile_velocity3)

# Syncwrite goal position
dxl_comm_result = groupSyncWrite2.txPacket()

# Clear syncwrite parameter storage
groupSyncWrite2.clearParam()

time.sleep(10)

####################################################################### ↓Force sensor
## Initialization
# initialize bytearrays
data = bytearray(36)
Fx_ = bytearray(4)
Fy_ = bytearray(4)
Fz_ = bytearray(4)
Tx_ = bytearray(4)
Ty_ = bytearray(4)
Tz_ = bytearray(4)

# ATI start command: 0x0002
# ATI stop command: 0x0000

start_command = str.encode(chr(18))+str.encode(chr(52))+str.encode(chr(00))+str.encode(chr(0o2))+str.encode(chr(00))+str.encode(chr(00))+str.encode(chr(00))+str.encode(chr(00))
stop_command = str.encode(chr(18))+str.encode(chr(52))+str.encode(chr(00))+str.encode(chr(00))+str.encode(chr(00))+str.encode(chr(00))+str.encode(chr(00))+str.encode(chr(0o1))


# ATI NET-FT
atiAddress = ('192.168.1.1',49152)
# create socket
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
# connect to NetBox
sock.connect(atiAddress)
print('Socket opened')

path = 'NetFT/'
print("Logging to path ", path)
basename = input('Enter file name: ')
    
# start streaming FT-data by sending start command
#sock.sendto(start_command,atiAddress)
timeOffset = time.time();

#######################################################↑ Force sensor
sock.sendto(start_command,atiAddress)

with open(path+basename+'.csv','w') as logFile:
    logFile.write('Time,Fx,Fy,Fz,cur_1,cur_2,cur_3\n')#depends on record data
    #logFile.write('Fx,Fy,Fz,cur_1,cur_2,cur_3\n')#depends on record data
    #try:
    #   print("logging data....")

    tic()

    while 1:
                # Get Force Data
                #logFile.write('%f,'%t) # timestamp
        
        t = time.time() - startTime_for_tictoc
        
        if t >= 0 and t <= 18:
            dxl1_goal_position = int(w * t / 360 * 4096 + ini_W_position)
            dxl2_goal_position = int(ini_C_position)
            dxl3_goal_position = int(ini_S_position)

        # Allocate goal position value into byte array
            param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl1_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl1_goal_position))]
            param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position))]
            param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(dxl3_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl3_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl3_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl3_goal_position))]

        # Add Dynamixel goal position value to the Syncwrite storage
            dxl_addparam_result = groupSyncWrite1.addParam(DXL1_ID, param_goal_position1)
            dxl_addparam_result = groupSyncWrite1.addParam(DXL2_ID, param_goal_position2)
            dxl_addparam_result = groupSyncWrite1.addParam(DXL3_ID, param_goal_position3)

        # Syncwrite goal position
            dxl_comm_result = groupSyncWrite1.txPacket()

        # Clear syncwrite parameter storage
            groupSyncWrite1.clearParam()
            d = sock.recvfrom_into(data,36)
            for i in range(4):
                Fx_[i] = data[12+i]
                Fy_[i] = data[16+i]
                Fz_[i] = data[20+i]
                Tx_[i] = data[24+i]
                Ty_[i] = data[28+i]
                Tz_[i] = data[32+i]
        # convert binary data to float
            Fx = unpack('!i',Fx_)
            Fy = unpack('!i',Fy_)
            Fz = unpack('!i',Fz_)
            Tx = unpack('!i',Tx_)
            Ty = unpack('!i',Ty_)
            Tz = unpack('!i',Tz_)
            
            logFile.write('%f,%f,%f,%f,%d'%(t,Fx[0]/float(1000000),Fy[0]/float(1000000),Fz[0]/float(1000000),0))
        # Syncread present position
            
            #dxl_comm_result = groupSyncRead.txRxPacket()           
            
            # Get Dynamixel#1 #2 #3 present position value
            dxl1_present_position = groupSyncRead.getData(DXL1_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
            dxl2_present_position = groupSyncRead.getData(DXL2_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
            dxl3_present_position = groupSyncRead.getData(DXL3_ID, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
            
            #logFile.write('%f,%f,%f'%(dxl1_present_position,dxl2_present_position,dxl3_present_position))
            
            #print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl1_goal_position, dxl1_present_position, DXL2_ID, dxl2_goal_position, dxl2_present_position, DXL3_ID, dxl3_goal_position, dxl3_present_position))
            
                
            #logFile.write('%f,%f,%f,%f,%f,%f,%f'%(t,Fx[0]/float(1000000),Fy[0]/float(1000000),Fz[0]/float(1000000),dxl1_present_position,dxl2_present_position,dxl3_present_position))
 
            logFile.write('\n')
        
        if t > 18:
            logFile.close()
            sock.sendto(stop_command,atiAddress)
                
# Clear syncread parameter storage
    groupSyncRead.clearParam()

#logFile.close()
#sock.sendto(stop_command,atiAddress)

# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#3 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()

#logData(path = 'NetFT/')
