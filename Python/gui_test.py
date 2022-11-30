#!/usr/bin/python

import tkinter
root = tkinter.Tk()
def hello():
    print("Hello")
zeroButton = tkinter.Button(root,text="Zero",command=hello)
zeroButton.pack(side="top")
root.mainloop()