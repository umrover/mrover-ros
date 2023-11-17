# A simple datalogging example with graphic output
# using MatPlotLib.
#
# This example expects receiving numerical values,
# terminated by CR+LF, from the serial port.
#
# The port is checked for new data every 1000ms.
# For a shorter interval, update the "interval"
# parameter in the FuncAnimation call.
#
# Author: Roger Meier, 04-30-2020
# CoolTerm version: 1.8.0

from random import randint
import re
import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import time
import CoolTerm
s = CoolTerm.CoolTermSocket()

# Get the ID of the first open window
ID = s.GetWindowID(0)
if ID < 0:
    print("No open windows")
    sys.exit()

# Connect to the serial port
if not s.Connect(ID):
    print("Not Connected")
    sys.exit()


# initialize the figure
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = [] # list to hold time stamps
ys = [] # list to hold received data values
t0 = time.time()


# Start logging
print("Listening to CoolTerm. Close the figure to exit.")

# This function is called periodically from FuncAnimation
def animate(i, xs, ys):

    s.Poll(ID)
    rcv = s.LookAhead(ID) # see what's been received so far
    pos = [m.end() for m in re.finditer('\r\n', rcv)]
    if len(pos) > 0:
        newData = True
        t1 = time.time()
        # we have received data
        # get everything up to the last CR+LF
        rcv = s.Read(ID, pos[-1])
        items = rcv.split('\r\n')
        for i in range(0,len(items)-1):
            # checking data for validity
            try:
                y = float(items[i])
            except:
                pass # ignore the current item
            else:
                # Add x and y to lists
                delta_t = t1 - t0
                xs.append(delta_t)
                ys.append(float(items[i]))
                print(xs[-1], ys[-1])
            
    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    #plt.subplots_adjust(bottom=0.30)
    plt.title('CoolTerm Data Logger')
    plt.ylabel('Data')

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
plt.show()
