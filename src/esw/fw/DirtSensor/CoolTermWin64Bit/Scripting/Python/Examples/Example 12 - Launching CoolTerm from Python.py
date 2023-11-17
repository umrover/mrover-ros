# CHECK IF COOLTERM IS RUNNING AND LAUNCH IT IF NOT
# =================================================
#
# Author: Roger Meier, 09-27-2022
# CoolTerm version: 2.0.0

# Set app and path names
import os
import platform
match platform.system(): # requires Python 3.10 or newer
    case "Darwin": # macOS
        appName = "CoolTerm"
        appPath = "/Applications/"
        # https://stackoverflow.com/questions/28845242/how-to-open-an-application-in-mac-os-using-python
    case "Windows":
        appName = "CoolTerm.exe"
        appPath = "C:\\Program Files\\CoolTermWin\\"
    case "Linux":
        appName = "CoolTerm"
        appPath = "/home/apps/CoolTermLinux/"

# Check if CoolTerm is running
import psutil
isRunning = appName in (i.name() for i in psutil.process_iter())

# Launch CoolTerm
import subprocess
import time
if not isRunning:
    print("Launching CoolTerm...")
    if platform.system() == "Darwin":
        os.system("open " + appPath + appName + ".app")
    else:
        subprocess.Popen(appPath + appName)
    time.sleep(5)
else:
    print("CoolTerm is already running.")
    
# ------------------------------------------------------

# Connect to CoolTerm
import CoolTerm

s = CoolTerm.CoolTermSocket()

# ------------------------------------------------------

# Do stuff with CoolTerm here:






# ------------------------------------------------------

# Disconnect from CoolTerm
s.Close()