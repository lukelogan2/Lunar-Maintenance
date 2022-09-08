
#Communication code for Pi to Arduinos
import serial
from time import sleep
import struct

#Set address
ser = serial.Serial('/dev/ttyACM0', 115200)
arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1)
#Wait for connection to complete
sleep(3)

#--------------------------------------------------------------------------
def writeBlock(x, y):
    bus.write_i2c_block_data(address, x, [y])
    return -1

def writeArduino(x,y,z):
    arduino.write(struct.pack('>BBB',x,y,z))
    time.sleep(0.05)

def ReadfromArduino():
    while (ser.in_waiting > 0):
        try:
            data = arduino.readline().decode('utf-8').rstrip()
            return data
        except:
            print("Communication Error")
#--------------------------------------------------------------------------
#set Zero on Arduino
try:
    writeArduino(X,Y,Z)
    sleep(.1)
except:
    print("Communication Error")

#Location for spare/stow loccations <PLACEHOLDERS>
X_stow = 0
Y_stow = 55
Z_stow = 60
X_spare = 200
Y_spare = 55
Z_spare = 60
#default settings
c = 0 #Tells arduino what to do with end effector, 0 for nothing, 1 for remove valve, 2 for drop/stow valve
#Z_dflt = 50
#Main loop
while True:
    try:
        sleep(.5)
        #readReady = arduino.read()
        readReady = 1     
    except:
        print("Communication Error")
    if readReady == 1:
        X_user = input("X>")                 # user input for x-coordinate
        Y_user = input("Y>")                 # user input for y-coordinate
        Z_user = input("Z>")                 # user input for z-coordinate NOTE: We will want this to be a default value so user doesn't break anything, but left in for now for calibration
        #Z_user = Z_dflt
        cmd = input("Command 0 for only xyz movement, 1 for valve replacement procedure>")         # user input to set cmd flag for arduino, 0 for just coordinate movement, 1 to perform replacement

        #XYZ movment commands
        X = X_user
        Y = Y_user
        Z = Z_user
        if cmd ==1:  #Valve removal procedure
            c = 1 #Initiate Valve removal
        try:
            writeArduino(c,X,Y,Z)
            sleep(.1)
        except:
            print("Communication Error")
        #    
        if cmd == 1: #Valve removal procedure
            #Stow old valve
            X = X_stow
            Y = Y_stow
            Z = Z_stow
            c = 2   #Initiate Valve stow Procedure
            try:
                writeArduino(c,X,Y,Z)
                sleep(.1)
            except:
                print("Communication Error")

            #Spare
            X = X_spare
            Y = Y_spare
            Z = Z_spare
            c = 1  #Initiate Valve removal
            try:
                writeArduino(c,X,Y,Z)
                sleep(.1)
            except:
                print("Communication Error")
                
            #Replace  
            X = X_user
            Y = Y_user
            Z = Z_user
            c = 2 #Initiate Valve removal
            try:
                writeArduino(c,X,Y,Z)
                sleep(.1)
            except:
                print("Communication Error")
            #    
            
                
        readReady = 0                       #Check for edge case if readReady isn't reset on Arduino yet

#-------------------------------------------------------------
