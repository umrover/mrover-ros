# Responding to received data
#
# The scripts grabs received characters from the
# receive buffer. If a number character is received,
# it responds with a word corresponding to that number.
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

# Open the serial port
if not s.Connect(ID):
    print("Not connected")
    sys.exit()

print("Listening for numbers 0..9. Press CTRL-C to abort")

numbers = ['zero','one','two','three','four','five','six','seven','eight','nine']

try:

    while True: # endless loop
        i = 0
        while i <= 0:
            # Poll the port
            s.Poll(ID)
            # see if any data has arrived
            i = s.BytesAvailable(ID)
            time.sleep(0.2)

        # Read the last byte from the receive buffer
        d = s.Read(ID, 1)
        n = 0
        try:
            n = int(d)
        except:
            pass

        if n >= 0 and n <= 9:
            s.WriteLine(ID, numbers[n])
            
except KeyboardInterrupt:
    pass
    
# Close the port
s.Disconnect(ID)

# Disconnect from CoolTerm
s.Close()