# Communication code for Pi to Arduinos
import serial
from time import sleep

try:
    arduino = serial.Serial(port='COM7', baudrate=115200)
    sleep(3)
except:
    print("Failed Serial")

########################################################
# Read Serial Input from Arduino
########################################################
def readSerial():
    while (arduino.in_waiting > 0):
        try:
            line = arduino.readline().decode('utf-8').rstrip()
            print("serial output : ", line)
        except:
            print("Communication Error")

########################################################
# Write Serial Data to Arduino
########################################################
def sendSerial(message):
    try:
        arduino.write(message.encode())
        print("Writing " + message)
    except:
        print("Serial Message Failed to Send")

print('Serial Connection Succeeded')

########################################################
## Function to set the tool position in the gantry coordinate space
# The origin is defined as the lower front-left corner of the frame
#   x = horizontal position in mm (left is negative, right is positive)
#   y = vertical position in mm (up is positive, down is negative)
#   z = inward position to move the tool in mm (forward is positive, backward is negative)
#   c = tool position in degrees (0 degrees is open, 100 degrees is closed, 180 degrees is tightly closed)
########################################################
def setPosition(x,y,z,c):
    message = "{:03d}{:03d}{:03d}{:03d}".format(x,y,z,c)
    sendSerial(message)

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

#open_tool()
#close_tool()

while True:
    print("Set coordinate and tool position of the gantry:\n")
    x = int(input("X = \n"))
    y = int(input("Y = \n"))
    z = int(input("Z = \n"))
    c = int(input("Claw position (0 = open, 100 = closed, 180 = tightly closed):\n"))
    setPosition(x,y,z,c)
    sleep(3)
    readSerial()

arduino.close()