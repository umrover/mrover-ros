# Demonstrating sending received characters to the console in real time.
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
if s.Connect(ID):

    print("Listening to CoolTerm. Press CTRL-C to exit.")

    try:

        while True: # endless loop
            i = 0
            while i <= 0:
                # Poll the port
                s.Poll(ID)
                # see if any data has arrived
                i = s.BytesAvailable(ID)
                time.sleep(0.2)

            # Read the data from the buffer
            d = s.ReadAll(ID)

            # print to console
            print(d, end = '')

    except KeyboardInterrupt:
        pass
        
    # Close the port
    s.Disconnect(ID)

    # Disconnect from CoolTerm
    s.Close()

else:
    print("Not Connected")
