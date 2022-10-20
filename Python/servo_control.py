# Communication code for Pi to Arduinos
import serial
from time import sleep

try:
    arduino = serial.Serial(port='COM7', baudrate=115200, timeout=2)
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
    pos = input("Servo Position: ")
    sendSerial(str(pos))

arduino.close()