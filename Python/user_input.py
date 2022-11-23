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
    print(x,y,z,c,s)