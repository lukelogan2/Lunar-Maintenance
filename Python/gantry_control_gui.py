# Communication code for Pi to Arduinos
import serial
from time import sleep
import tkinter
from tkinter import messagebox

##################################################################
# Global Variables
##################################################################
global stowPos, rSolPos, solPos, stashPos, clampAngle, zeroed
solPos = (81, 283, 27)
rSolPos = (81, 283, 27)
stowPos = (329, 280, 27)
stashPos = (220, 103, 27)
clampClosed = 85
clampOpen = 0
zeroed = False

##################################################################
# GUI Variables
##################################################################
root = tkinter.Tk()
# Initialize tkinter window with dimensions 100x100
width= root.winfo_screenwidth()
height= root.winfo_screenheight()
#setting tkinter window size
root.geometry("%dx%d" % (width, height))
root.title("Gantry Control")
x_var = tkinter.IntVar()
y_var = tkinter.IntVar()
z_var = tkinter.IntVar()
c_var = tkinter.IntVar()
s_var = tkinter.IntVar()

##################################################################
# Setup Serial Communication with Arduino
##################################################################
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

############################################################################################################
## Function to set the tool position in the gantry coordinate space
# The origin is defined as the lower front-left corner of the frame
#   x = horizontal position in mm (left is negative, right is positive)
#   y = vertical position in mm (up is positive, down is negative)
#   z = inward position to move the tool in mm (forward is positive, backward is negative)
#   c = tool position in degrees (0 degrees is open, 100 degrees is closed, 180 degrees is tightly closed)
#   s = solenoid valve position (0 = open, 1 = closed)
###########################################################################################################
def setPosition(x,y,z,c,s):
    message = "{:06d}{:06d}{:06d}{:06d}{:01d}".format(x,y,z,c,s)
    sendSerial(message)

#########################################################################################################
## Function to set the tool position and block other movements until the current movement is complete
#########################################################################################################
def setPositionBlocking(x,y,z,c,s):
    message = "{:06d}{:06d}{:06d}{:06d}{:01d}".format(x, y, z, c, s)
    sendSerial(message)
    doneFlag = False
    while doneFlag == False:
        msg = readSerial()
        if msg == "done":
            doneFlag = True

#########################################################################################################
## Function to set the gantry position via the GUI
#########################################################################################################
def manualPosition():
    x = x_var.get()
    y = y_var.get()
    z = z_var.get()
    c = c_var.get()
    s = s_var.get()
    setPosition(x, y, z, c, s)
    '''
    if (not zeroed and c >= 0 and c <= 180 and (s==0 or s==1)):
        setPosition(x,y,z,c,s)
    else:
        if (zeroed and x>=0 and y>=0 and z>=0 and c >= 0 and c <= 180 and (s==0 or s==1)):
            setPosition(x, y, z, c, s)
        else:
            tkinter.messagebox.showwarning(title="Warning", message="Invalid Input")
    '''
########################################################################
## Remove the stow solenoid and place it in the system location
########################################################################
def replaceSolenoid():
    global stowPos, solPos, clampClosed, clampOpen
    setPositionBlocking(stowPos[0], stowPos[1], 10, clampOpen, 0)  # Move in front of the solenoid
    sleep(3)
    setPositionBlocking(stowPos[0], stowPos[1], stowPos[2], clampOpen, 0)  # The tool is in position
    sleep(3)
    setPositionBlocking(stowPos[0], stowPos[1], stowPos[2], clampClosed, 0)  # Clamp down on the solenoid
    sleep(3)
    setPositionBlocking(stowPos[0], stowPos[1], 10, clampClosed, 0)  # Move the solenoid out
    sleep(3)
    setPositionBlocking(solPos[0], solPos[1], 10, clampClosed, 0)  # Move to the X, Y position of the stow location
    sleep(3)
    setPositionBlocking(solPos[0], solPos[1], 17, clampClosed, 0)  # Move the Z-axis tool closer
    sleep(3)
    setPositionBlocking(solPos[0], solPos[1], 22, clampClosed, 0)  # Move the Z-axis tool closer
    sleep(3)
    setPositionBlocking(solPos[0], solPos[1], solPos[2], clampClosed, 0)  # The tool is in position
    sleep(3)
    setPositionBlocking(solPos[0], solPos[1]+1, solPos[2], clampClosed, 0)  # The tool is in position
    sleep(3)
    setPositionBlocking(solPos[0], solPos[1], solPos[2], clampOpen, 0)  # Open the clamp
    sleep(3)
    setPositionBlocking(solPos[0], solPos[1], 15, clampOpen, 0)  # Move the Z-axis tool out
    sleep(3)

########################################################################
## Remove the system solenoid and place it in the trash location
########################################################################
def removeSolenoid():
    global stowPos, rSolPos, stashPos, clampClosed, clampOpen
    setPositionBlocking(rSolPos[0], rSolPos[1], 10, clampOpen, 0)  # Move in front of the solenoid
    sleep(3)
    setPositionBlocking(rSolPos[0], rSolPos[1], rSolPos[2], clampOpen, 0)  # The tool is in position
    sleep(3)
    setPositionBlocking(rSolPos[0], rSolPos[1], rSolPos[2], clampClosed, 0)  # Clamp down on the solenoid
    sleep(3)
    setPositionBlocking(rSolPos[0], rSolPos[1], 10, clampClosed, 0)  # Move the solenoid out
    sleep(3)
    setPositionBlocking(stashPos[0], stashPos[1], 10, clampClosed, 0)  # Move to the X, Y position of the stow location
    sleep(3)
    setPositionBlocking(stashPos[0], stashPos[1], 16, clampClosed, 0)  # Move the Z-axis tool closer
    sleep(3)
    setPositionBlocking(stashPos[0], stashPos[1], 19, clampClosed, 0)  # Move the Z-axis tool closer
    sleep(3)
    setPositionBlocking(stashPos[0], stashPos[1], 24, clampClosed, 0)  # Move the Z-axis tool closer
    sleep(3)
    setPositionBlocking(stashPos[0], stashPos[1], stashPos[2], clampClosed, 0)  # The tool is in position
    sleep(3)
    setPositionBlocking(stashPos[0], stashPos[1]+1, stashPos[2], clampClosed, 0)  # The tool is in position
    sleep(3)
    setPositionBlocking(stashPos[0], stashPos[1], stashPos[2], clampOpen, 0)  # Open the clamp
    sleep(3)
    setPositionBlocking(stashPos[0], stashPos[1], 15, clampOpen, 0)  # Move the Z-axis tool out
    sleep(3)

########################################################
# Function to zero the motors
########################################################
def zeroRoutine():
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
    zeroed = True


#####################################################################################################
# GUI Setup
#####################################################################################################

# configure the grid
root.columnconfigure(0, weight=1)
root.columnconfigure(1, weight=1)
top_label = tkinter.Label(root,text="Gantry Control User Interface",font=('calibre',24,'bold'),bg="green",fg="white")
x_label = tkinter.Label(root, text = "X Position [mm]", font=('calibre',24,'bold'))
y_label = tkinter.Label(root, text = "Y Position [mm]", font=('calibre',24,'bold'))
z_label = tkinter.Label(root, text = "Z Position [mm]", font=('calibre',24,'bold'))
c_label = tkinter.Label(root, text = "Clamp Position [0 - 180 deg]", font=('calibre',24,'bold'))
s_label = tkinter.Label(root, text = "Solenoid Position [Open = 0, Closed = 1]", font=('calibre',24,'bold'))
x_entry = tkinter.Entry(root, textvariable=x_var, justify=tkinter.CENTER, font=('calibre',24,'bold'))
y_entry = tkinter.Entry(root, textvariable=y_var, justify=tkinter.CENTER, font=('calibre',24,'bold'))
z_entry = tkinter.Entry(root, textvariable=z_var, justify=tkinter.CENTER, font=('calibre',24,'bold'))
c_entry = tkinter.Entry(root, textvariable=c_var, justify=tkinter.CENTER, font=('calibre',24,'bold'))
s_entry = tkinter.Entry(root, textvariable=s_var, justify=tkinter.CENTER, font=('calibre',24,'bold'))
zeroBtn = tkinter.Button(root,text="Zero Motors",font=('calibre',24,'bold'),bg="Red",command=zeroRoutine)
subBtn = tkinter.Button(root,text="Send Coordinates",font=('calibre',24,'bold'),bg="Red",command=manualPosition)
removeBtn = tkinter.Button(root,text="Remove Solenoid",font=('calibre',24,'bold'),bg="Red",command=removeSolenoid)
replaceBtn = tkinter.Button(root,text="Replace Solenoid",font=('calibre',24,'bold'),bg="Red",command=replaceSolenoid)
top_label.grid(row=0, column=0, columnspan=2,sticky=tkinter.EW)
x_label.grid(row=1,column=0,sticky=tkinter.EW)
x_entry.grid(row=1,column=1,sticky=tkinter.EW,padx=50,pady=10)
y_label.grid(row=2,column=0,sticky=tkinter.EW)
y_entry.grid(row=2,column=1,sticky=tkinter.EW,padx=50,pady=10)
z_label.grid(row=3,column=0,sticky=tkinter.EW)
z_entry.grid(row=3,column=1,sticky=tkinter.EW,padx=50,pady=10)
c_label.grid(row=4,column=0,sticky=tkinter.EW)
c_entry.grid(row=4,column=1,sticky=tkinter.EW,padx=50,pady=10)
s_label.grid(row=5,column=0,sticky=tkinter.EW)
s_entry.grid(row=5,column=1,sticky=tkinter.EW,padx=50,pady=10)
subBtn.grid(row=6,column=0,columnspan=2,sticky=tkinter.EW,padx = 50, pady = 10)
zeroBtn.grid(row=7,column=0,columnspan=2, padx = 50, pady = 10, sticky=tkinter.EW)
removeBtn.grid(row=8,column=0,columnspan=2, padx = 50, pady = 10, sticky=tkinter.EW)
replaceBtn.grid(row=9,column=0,columnspan=2, padx = 50, pady = 10, sticky=tkinter.EW)
root.mainloop()

arduino.close()
