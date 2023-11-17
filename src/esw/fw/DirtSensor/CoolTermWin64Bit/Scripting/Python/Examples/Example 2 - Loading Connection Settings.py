# Loading CoolTerm connection settings
#
# Author: Roger Meier, 06-11-2020
# CoolTerm version: 1.8.0

import CoolTerm
import sys
s = CoolTerm.CoolTermSocket()

# Load connection settings file. Replace the path with the location of your settings file.
FilePath = "Macintosh HD:Users:Shared:Untitled_0.stc"
ID = s.LoadSetting(FilePath)
if ID < 0:
    print("The settings could not be loaded")
    sys.exit()

input("The settings have been loaded successfully. Press ENTER to close the window")

# Close the window
s.CloseWindow(ID)

# Disconnect from CoolTerm
s.Close()


