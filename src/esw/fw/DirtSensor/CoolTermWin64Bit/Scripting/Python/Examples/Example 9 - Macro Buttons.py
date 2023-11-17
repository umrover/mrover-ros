# Creating Macro Buttons using tkinter
#
# Enter the macro text into the input field and
# press the button next to it to send it using the
# frontmost CoolTerm window.
# use /n, /r, /t to send LF, CR, or TAB, respectively
#
# This example creates 2 buttons. You may add as many
# buttons as you like.
#
# Author: Roger Meier, 06-07-2020
# CoolTerm version: 1.8.0

import CoolTerm
import sys
import os
import tkinter
from tkinter import messagebox

# --------------------------------------------------------
# CoolTerm Setup
# --------------------------------------------------------
s = CoolTerm.CoolTermSocket()

def send(txt):
    # Get ID of frontmost window
    ID = s.GetFrontmostWindow()
    if ID < 0:
        messagebox.showwarning("CoolTerm Warning","There are no open windows")
        return
    if not s.IsConnected(ID):
        messagebox.showwarning("CoolTerm Warning","Port is not open")
        return        
    txt = txt.replace("\\n","\n") # replace "\n" with LF
    txt = txt.replace("\\r","\r") # replace "\r" with CR
    txt = txt.replace("\\t","\t") # replace "\t" with TAB
    s.Write(ID, txt)
    
# --------------------------------------------------------
# GUI Setup
# --------------------------------------------------------
w = tkinter.Tk()
w.title("CoolTerm Macros")

# BUTTON 1
def send1():
    send(txt1.get())
txt1 = tkinter.Entry(w, width = 20)
txt1.grid(column=0, row=0)
btn1 = tkinter.Button(w, text = "Send", command = send1) 
btn1.grid(column=1, row=0)
txt1.focus()

# BUTTON 2
def send2():
    send(txt2.get())
txt2 = tkinter.Entry(w, width = 20)
txt2.grid(column=0, row=1)
btn2 = tkinter.Button(w, text = "Send", command = send2) 
btn2.grid(column=1, row=1)

# CLOSE BUTTON
btnClose = tkinter.Button(w, text = "Close Window", command = w.destroy) 
btnClose.grid(row=3)

w.mainloop() # Code execution will stop here until w is closed
print("Done")

