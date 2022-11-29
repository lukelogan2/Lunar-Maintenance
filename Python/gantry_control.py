# Communication code for Pi to Arduinos
import serial
from time import sleep

global stowPos, solPos, clampAngle
stowPos = (82, 278, 27)
solPos = (332,274,27)
clampClosed = 85
clampOpen = 0

try:
    arduino = serial.Serial(port='COM7', baudrate=115200)
    sleep(3)
    print('Serial Connection Succeeded')
except:
    print("Failed Serial")

xcur = 0
ycur = 0
zcur = 0
ccur = 0
scur = 0
########################################################
# Read Serial Input from Arduino
########################################################
def readSerial():
    while (arduino.in_waiting > 0):
        try:
            line = arduino.readline().decode('utf-8').rstrip()
            print("serial output : ", line)
            return(line)
        except:
            print("Communication Error")
            return("-1")

########################################################
# Write Serial Data to Arduino
########################################################
def sendSerial(message):
    try:
        arduino.write(message.encode())
        print("Writing " + message)
    except:
        print("Serial Message Failed to Send")

########################################################
## Function to set the tool position in the gantry coordinate space
# The origin is defined as the lower front-left corner of the frame
#   x = horizontal position in mm (left is negative, right is positive)
#   y = vertical position in mm (up is positive, down is negative)
#   z = inward position to move the tool in mm (forward is positive, backward is negative)
#   c = tool position in degrees (0 degrees is open, 100 degrees is closed, 180 degrees is tightly closed)
#   s = solenoid valve position (0 = open, 1 = closed)
########################################################
def setPosition(x,y,z,c,s):
    message = "{:06d}{:06d}{:06d}{:06d}{:01d}".format(x,y,z,c,s)
    sendSerial(message)

################################################################
## Function to set the tool position and block other movements until the current movement is complete
###############################################################
def setPositionBlocking(x,y,z,c,s):
    message = "{:06d}{:06d}{:06d}{:06d}{:01d}".format(x, y, z, c, s)
    sendSerial(message)
    doneFlag = False
    while doneFlag == False:
        msg = readSerial()
        if msg == "done":
            doneFlag = True

########################################################################
## Remove the solenoid and place it in the stow location
########################################################################
def removeSolenoid():
    global stowPos, solPos, clampClosed, clampOpen
    setPositionBlocking(solPos[0], solPos[1], 0, clampOpen, 0)  # Move in front of the solenoid
    sleep(3)
    setPositionBlocking(solPos[0], solPos[1], solPos[2], clampOpen, 0)  # The tool is in position
    sleep(3)
    setPositionBlocking(solPos[0], solPos[1], solPos[2], clampClosed, 0)  # Clamp down on the solenoid
    sleep(3)
    setPositionBlocking(solPos[0], solPos[1], 10, clampClosed, 0)  # Move the solenoid out
    sleep(3)
    setPositionBlocking(stowPos[0], stowPos[1], 10, clampClosed, 0)  # Move to the X, Y position of the stow location
    sleep(3)
    setPositionBlocking(stowPos[0], stowPos[1], 17, clampClosed, 0)  # Move the Z-axis tool closer
    sleep(3)
    setPositionBlocking(stowPos[0], stowPos[1], 22, clampClosed, 0)  # Move the Z-axis tool closer
    sleep(3)
    setPositionBlocking(stowPos[0], stowPos[1], stowPos[2], clampClosed, 0)  # The tool is in position
    sleep(3)
    setPositionBlocking(stowPos[0], stowPos[1], stowPos[2], clampOpen, 0)  # Open the clamp
    sleep(3)
    setPositionBlocking(stowPos[0], stowPos[1], 10, clampOpen, 0)  # Move the Z-axis tool out
    sleep(3)


def open_tool():
    pos = 180
    while pos >= 0:
        sendSerial(str(pos))
        sleep(1)
        readSerial()
        pos = pos - 30

def close_tool():
    pos = 0
    while pos <= 180:
        sendSerial(str(pos))
        sleep(1)
        readSerial()
        pos = pos + 30

########################################################
# Start by letting the user decide to zero
# ctl - Determined the control scheme of the program
#   "man" = manual control
#   "auto" = automated control
########################################################
def zeroRoutine(ctl):
    if ctl == "man":
        zero = input("Zero Motors? (y/n)")
    else:
        zero = "y"
    if zero == "y":
        message = "zero"
        sendSerial(message)
        # Wait for the motor to zero
        zeroX = False
        zeroY = False
        zeroZ = False
        while (not zeroX) or (not zeroY) or (not zeroZ):
            msg = readSerial()
            if msg == "zeroX":
                zeroX = True
                print(msg)
            if msg == "zeroY":
                zeroY = True
                print(msg)
            if msg == "zeroZ":
                zeroZ = True
                print(msg)
    else:
        message = "nozero"
        sendSerial(message)

#############################################################################################
# Let the user define positions to move the robot tool for the remainder of the program
#############################################################################################
def runProgram():
    while True:
        print("Set coordinate and tool position of the gantry:\n")
        x = -1
        while (type(x) != int or x < 0):
            try:
                x = int(input("X = \n"))
                if (x < 0):
                    print("Make sure X >= 0")
            except:
                print("X must be an integer")
        y = -1
        while (type(y) != int or y < 0):
            try:
                y = int(input("Y = \n"))
                if (y < 0):
                    print("Make sure Y >= 0")
            except:
                print("Y must be an integer")
        z = -1
        while (type(z) != int or z < 0):
            try:
                z = int(input("Z = \n"))
                if (z < 0):
                    print("Make sure Z >= 0")
            except:
                print("Z must be an integer")
        c = -1
        while (type(c) != int or c < 0 or c > 180):
            try:
                c = int(input("Input Claw Position (0 = open, 100 = closed, 180 = tightly closed):\n"))
                if (c < 0 or c > 180):
                    print("Make sure 0 <= claw pos <= 180")
            except:
                print("Claw position must be an integer")
        s = -1
        while (type(s) != int or (s != 0 and s != 1)):
            try:
                s = int(input("Input Solenoid Position (0=open, 1=closed):\n"))
                if (s != 0 and s != 1):
                    print("Make sure the input is 0 or 1")
            except:
                print("Solenoid position must be an integer")
        setPosition(x,y,z,c,s)
        #readSerial()


#####################################################################################################
# Main Program Script
#####################################################################################################
control = input("Manual (m) or Automated (a)?")
if (control == "m"):
    zeroRoutine("manual")
    runProgram()
elif (control == "a"):
    zeroRoutine("auto")
    removeSolenoid()
    runProgram()

arduino.close()
