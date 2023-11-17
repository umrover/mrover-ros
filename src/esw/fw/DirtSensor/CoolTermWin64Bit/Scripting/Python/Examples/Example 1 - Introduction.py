# A short script to demonstrate CoolTerm scripting
#
# Author: Roger Meier, 09-22-2022
# CoolTerm version: 2.0.0

import CoolTerm
import sys
s = CoolTerm.CoolTermSocket()

# Check if there are any open windows
count = s.WindowCount()

if count > 0:
    # Get the ID of the frontmost open window
    ID = s.GetFrontmostWindow()
else:
    # There are no open windows (this is only possible on macOS)
    # Open a new window
    ID = s.NewWindow()
    
# Open the serial port
if s.Connect(ID):
    
    # Send some data
    s.Write(ID, "Hello CoolTerm")
    s.WriteLine(ID, ", how are you?")
    s.WriteLine(ID, "At the next prompt, send me some data.")
    answer = input("Send some data to CoolTerm. Press ENTER to continue.")

    # Poll the port and move any data from the serial receive
    # buffer to the CoolTerm receive buffer
    s.Poll(ID)

    # See how much data is available
    i = s.BytesAvailable(ID)

    # Get a copy of the data in the receive buffer
    d1 = s.LookAhead(ID)
    print("I have received",i,"characters:",d1)

    # Read 5 bytes from the buffer and display as hex string
    d2 = s.ReadHex(ID, 5)
    i = s.BytesAvailable(ID)
    print("I grabbed the following 5 bytes from the buffer:",d2)
    print("There are",i,"characters left in the buffer.")

    # Read what's left in the buffer
    d1 = s.ReadAll(ID)
    print("I grabbed the following remaining characters from the buffer:",d1)

    # Close the port
    s.Disconnect(ID)

    # Disconnect from CoolTerm
    s.Close()

else:
    index = s.IndexOfWindowID(ID)
    name = s.WindowName(index)
    port = s.GetParameter(ID, "Port")
    print("Could not Connect to " + name)
    print("Make sure port " + port + " is available")
