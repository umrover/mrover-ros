#!/usr/bin/env python3
import subprocess
import rospy
import rospkg
import os

rospack = rospkg.RosPack()
rospy.init_node("gui")
guiPath = os.path.join(rospack.get_path("mrover"), "src/teleop/gui/")
os.chdir(guiPath)
bashCommand = "yarn run serve"
process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
output, error = process.communicate()
