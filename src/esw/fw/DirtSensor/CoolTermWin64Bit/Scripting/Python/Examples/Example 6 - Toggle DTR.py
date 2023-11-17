# This script demonstrates changing the state of hardware
# handshake lines. While the script is running, you can see
# the DTR LED in the CoolTerm window blinking.
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

    print("Toggling DTR. Press CTRL-C to exit.")

    try:

        while True: # endless loop
            s.SetDTR(ID, not s.GetDTR(ID))
            time.sleep(0.5)
           
    except KeyboardInterrupt:
        pass
        
    # Close the port
    s.Disconnect(ID)

    # Disconnect from CoolTerm
    s.Close()

else:
    print("Not Connected")
