# Demonstrating file capture
#
# Author: Roger Meier, 06-11-2020
# CoolTerm version: 1.8.0

import sys
import time
import CoolTerm
s = CoolTerm.CoolTermSocket()

# Get the ID of the first open window
ID = s.GetWindowID(0)
if ID < 0:
    print("No open windows")
    sys.exit()

# Start the file capture. Use a path with write access for this test.
FilePath = "/Users/Shared/capture.txt"
if not s.CaptureStart(ID, FilePath):
    print("Could not start capture")
    sys.exit()

# Open the serial port
if s.Connect(ID):

    # Turn on local echo
    s.SetParameter(ID, "LocalEcho", True)
    s.SetParameter(ID, "CaptureLocalEcho", True)
    s.SetParameter(ID, "CaptureWaitForTerminationString", False)

    # Send some data
    s.Write(ID, "Hello CoolTerm")

    # Stop the capture
    s.CaptureStop(ID)

    # Close the port
    s.Disconnect(ID)

    # Disconnect from CoolTerm
    s.Close()

else:
    print("Not Connected")
