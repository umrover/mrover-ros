# This script demonstrates changing the serial port of the
# current CoolTerm window.
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

# Making sure the port is closed
if s.IsConnected(ID):
    # Close the port
    s.Disconnect(ID)

# Get the number of ports
n = s.GetSerialPortCount()
print("Available Serial Ports:")

# Print a list of ports
for i in range(0,n):
    print(str(i)+":", s.GetSerialPortName(i))

# Select port
m = -1
while m < 0 or m >= n:
    m = int(input("Select Port (0.." + str(n-1) + "): "))

# Set the port
if s.SetCurrentSerialPort(ID, m):
    print("The port was set successfully")
else:
    print("The port could not be set")
    
# Disconnect from CoolTerm
s.Close()