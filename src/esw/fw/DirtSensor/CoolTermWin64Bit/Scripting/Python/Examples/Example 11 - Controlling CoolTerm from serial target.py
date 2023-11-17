# Controlling CoolTerm from the serial target
#
# Author: Roger Meier, 07-23-20
# CoolTerm version: 1.8.0

# This script demonstrates an easy way to control certain CoolTerm
# aspects from a connected serial target, such as an Arduino, etc.

# The concept is to wrap specific commands with specific markers (strings)
# that the script can recognize. It is important that the markers as well as
# the command string are chosen such that the resulting text string is not
# something that would occur in normal communication with the serial target.
begMarker = '!#' # Every command string from the target has to start with this
endMarker = '#!' # Every command string from the target has to end with this.

# In this example, we demonstrate how to start and stop file capture with specific
# strings sent from the target. This can easily be extended to other functions.
cmdStart = "@1"  # To start file capture, send !#@1#! from the target
cmdStop  = "@0"  # To stop file capture, send !#@0#! from the target

# The filename (or even whole file paths can be submitted in the same way
# e.g. "!#c:\Users\Public\capture.txt#!"
# This string will interpret any text between the command markers as a file name
# unless it's one of the defined commands above.
fileName = "" # initializing variable for the file path

# This script continuously monitors CoolTerm's receive buffer contents,
# i.e. whatever data currently resides in the CoolTerm window. It is therefore
# important to remember what command was received last.
lastCMD = "" # last command



import sys
import re # Regular Expressions
import CoolTerm
s = CoolTerm.CoolTermSocket()


# Get the ID of the first open window
ID = s.GetWindowID(0)
if ID < 0:
    print("No open windows")
    sys.exit()

print("Listening to CoolTerm. Press CTRL-C to exit.")

# Main Loop
try:
    while True: # endless loop
        # Poll the port
        s.Poll(ID)
        # Look at the contents of CoolTerm's receive buffer
        data = s.LookAhead(ID)
        # Inspect the data for command strings
        matches = re.findall(begMarker + '(.+?)' + endMarker, data)
        numMatches = len(matches) # number of command strings received
        if numMatches > 0:
            # get the most recent match
            cmd = matches[numMatches-1]
            # see if we received a new command
            if cmd != lastCMD:
                lastCMD = cmd
                # interpret the command
                if cmd == cmdStart: # start file capture
                    print("Received cmdStart")
                    if cmd != "":
                        if not s.CaptureStart(ID, fileName):
                            print("Could not start capture")
                    else:
                        print("Can't start file capture without a file name")
                elif cmd == cmdStop: #stop file capture
                    print("Received cmdStop")
                    s.CaptureStop(ID)
                else: # this must be a file name
                    fileName = cmd
                    print("Received File Name", fileName)
except KeyboardInterrupt:
    pass


# Disconnect from CoolTerm
s.Close()