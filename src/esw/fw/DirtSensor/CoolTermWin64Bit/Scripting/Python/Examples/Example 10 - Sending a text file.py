# Sending text file via CoolTerm
#
# Author: Roger Meier, 06-23-2020
# CoolTerm version: 1.8.0

import CoolTerm
s = CoolTerm.CoolTermSocket()

# Specify the absolute path to your text file here
filepath = "/Users/Shared/textfile.txt"

# Get the ID of the first open window
ID = s.GetFrontmostWindow()
if ID < 0:
    print("No open windows")
    sys.exit()

# Open the serial port
if s.Connect(ID):
    
    # send the file
    success = s.SendTextFile(ID, filepath)
    if not success:
        print("Could not send text file")
    
    # wait until it's done sending
    while s.BytesLeftToSend(ID) > 0:
        s.Poll(ID)
    
    print("Done sending!")

    # Close the port
    s.Disconnect(ID)

else:
    print("Not Connected")

