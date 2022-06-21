Code to control the drive motors using the ODrive Brushless Motor Controller 
----

### About
For the 2022 Mars Rover Rosie, we used 3 ODrive motor controllers to drive the wheels. odrives.py is the program
responsible for handling ODrive ROS messages recieved from the base station. config/odrive_config.yaml has the config info used
in odrives.py.

The program subscribes to a ROS topic to receive velocity. The states of the ODrive are **ArmedState**,
**DisconnectedState**, and **ErrorState.**

From the base station, velocity commands can be sent, as well as requests to view velocity estimate data,
current draw data, and the current state of the ODrive.
When the ODrive is armed it controls the speed of the motors in its closed-loop velocity control mode, getting data about 
the motors' current speed via the encoders integrated into the controller. 

Within in the program, most commands that can be found on the [ODrive website](https://odriverobotics.com/) have been abstracted by the Modrive class. Each state is its own class, and is instantiated and used by the ODriveBridge state machine. The main function then updates the state machine. 

The program uses multi-threading so that it can handle vel estimate/current draw data commands as well as speed/state
commands at the same time. Threading locks are used within the code to prevent the threads from running parts of the code 
simultaneously. 

### Topics - Subscriber
**Drive Velocity Command [Subscriber]** \
Messages: [ DriveVelCmd.msg ](https://github.com/umrover/mrover-ros/blob/main/msg/DriveVelCmd.msg) "drive_vel_cmd" \
Publishers: teleop \
Subscribers: odrives

### Topics - Publisher

**Drive Velocity Data [Publisher]** \
Messages: [ DriveVelData.msg ](https://github.com/umrover/mrover-ros/blob/main/msg/DriveVelData.msg) "drive_vel_data" \
Publishers: odrives \
Subscribers: gui

**Drive State Data [Publisher]** \
Messages: [ DriveStateData.msg ](https://github.com/umrover/mrover-ros/blob/main/msg/DriveStateData.msg) "drive_state_data" \
Publishers: odrives \
Subscribers: gui

### Watchdog Timeout Period 
1 second

### States

**DisconnectedState** - In this state the program is searching for the ODrive by its ID number. Once it has found 
the ODrive, the state will immediately change to Armed. 

**ArmedState** - In this state the ODrive will respond to velocity commands until instructed otherwise or errors out. 

**ErrorState** - In this state the ODrive has detected a system error and will reboot, going from Disconnected to Armed immediately.


### Usage 
This code is meant to be run on the NVIDIA Jetson TX2. For external testing follow the wiring up guidelines specified [here](https://docs.odriverobotics.com/#wiring-up-the-odrive)
on the ODrive webite. \
The program automatically starts up upon connecting to the rover. \
In order to drive the rover, use the joystick - making sure its connected to the base station, you have communications, and no ODrive errors are showing up. \

### Setting Up A New ODrive 
[Link to the most updated guide as of 2021](https://docs.google.com/document/d/1HmKRBJYMA4qaA--7KrE8OWiwN6Ck05E5CHhxUegaq0A/edit)
### Electrical Modifications
Typically the ODrive does not have very good Hall Filtering capabilities, so some electrical modifications must be made prior to 
our usage of it. Six 47nF capacitors will have to be soldered on to the ODrive as shown 
[here](https://discourse.odriverobotics.com/t/encoder-error-error-illegal-hall-state/1047/7?u=madcowswe).

### Getting The ID 
Each ODrive has a unique serial ID. In order to determine that this ID is, follow the steps on 
[this](https://docs.odriverobotics.com/#downloading-and-installing-tools) website on the hoverboard guide page. To activate odrivetool
 follow the steps below. You can do this on the base station or the Jetson. On the Jetson make sure the \
 odrive_bridge program on the Jetson is [deactivated](Debugging). \
`$ ssh mrover@10.1.0.1` \
`$ mrover` \
`$ cd ~/.mrover` \
`$ source build_env/bin/activate` \
`$ odrivetool` \
This will start up odrivetool, and after a few seconds *Connected to ODrive [ID number] as odrvX* should appear on the screen. \
Type \
`$ quit()` \
`$ deactivate` \
to get out of this state. \
In the config/odrive_config.yaml, look at the line that sets the IDs. 
Depending on which ODrive you replaced, change its ID to the new one. 
Rebuild the program. \

### Changing The USB Permissions
USB permissions when connecting the ODrive to the Jetson have to be modified before you can successfully communicate with the odrive. This only has to be done once. \
Make sure the ODrive is connected via USB and ssh into the Jetson. Type \
`$ lsusb` . From list find the idVendor, idProduct, and MODE of the odrive. It will be listed under the info for the InterBiometrics
device. \
`$ sudo vi /etc/udev/rules.d/50-myusb.rules` \
add \
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d32", GROUP="mrover", MODE="0666" \
 to the file \
 If this is on your virtual machine change the group name to be vagrant. \
 Restart the Jetson.

### Calibrating The ODrive 
Calibration must be done manually. Once again ssh into the Jetson and go to the .mrover folder 
and start running odrivetool. \
`$ ssh mrover@10.1.0.1` \
`$ mrover` \
`$ cd ~/.mrover` \
`$ source build_env/bin/activate` \
`$ odrivetool` \
The odrives should automatically connect. Using the odrvX.axisY (0 or 1 depending on the ODrive with the id) in place of m_axis, execute all of the following commands for axis0 and axis1. \
`$ odrvX.axis0.requestedstate = AXIS_STATE_FULL_CALIBRATION_SEQUENCE ` \
 The motor should beep and start calibrating now. If it does not go to the **Errors** section below. 
 Once it has finished, type \
`$ odrvX.axis0.motor.config.pre_calibrated = True ` \
`$ odrvX.axis0.encoder.config.pre_calibrated = True ` \
 Repeat these three commands for axis1 as well. Then type \
`$ odrvX.reboot()` 


### Off Nominal Behavior Handling

### ODrive Errors 
If the ODrive throws an error, odrive_bridge will tell it to reboot. The error will be displayed in the terminal and the state will be displayed as ErrorState. Depending on which ODrive this happens on, either the front or back motors will be unresponsive for a minute until the ODrive finishes rebooting. The state will automatically change from DisconnectedState to Armed upon completion.

### USB Disconnection 
 If the ODrive is disconnected from the USB port, odrive_bridge will print "ODrive has been unplugged" to the terminal, and will change the state to DisconnectedState until the connection is reestablished. At that point the state will automantically change to Armed.  
 
### Debugging (on the Jetson)
First make sure that the odrives.py is not running.
In order to see if there are any ODrive errors, ssh into the Jetson \
`$ ssh mrover@10.1.0.1` \
`$ mrover` \
The terminal should now display that you are in mrover@mrover-Jetson  \
`$ cd ~/.mrover` \
`$ source bin/activate` \
`$ odrivetool` \
`$ dump_errors(odrv0)` \
`$ dump_errors(odrv1)` 

## Common Errors 

### ODrive is not responding to calibration 
At this point you should be in odrivetool, if not, follow steps above to get there. \
`$ dump_errors(odrvX, True)` \
`$ dump_errors(odrvX, True)` \
If an `ENCODER_HALL_ERROR` shows up only the first time, you are good to try calibration again. If no errors show up at all,
or if the error persists, re-check your wiring.

### Unexplained USB failures
For running on the Jetson nano, if you are getting usb suspend errors, \
```sudo vi /boot/extlinux/extlinux.conf``` \
At the end of APPEND add ```usbcore.autosuspend=-1``` \
With a space between the end of APPEND and the start of 'usbcore..' \
Then reboot. \
Upon reboot check ```cat /proc/cmdline``` to verify the change was put into place
** this does not work on the xavier due to linux boot reasons **

Also make sure nothing else is on the usb bus (including the wifi chip)

### USB Forwarding on VM 
Make sure the ODrive is connected via USB and type \
`$ lsusb` . From list find the idVendor, idProduct, and MODE of the odrive. It will be listed under the info for the InterBiometrics 
device. \
`$ sudo vi /etc/udev/rules.d/99-usb-serial.rules` \
`$ SUBSYSTEMS=="usb", ATTRS{idVendor}=="[__idVendor__]", ATTRS{idProduct}=="[__idProduct__]", GROUP="mrover", MODE="[__MODE__]" ` \
 Restart the VM.
 
### ERROR_ILLEGAL_HALL_STATE
`dump_errors(odrvX, True)` \
If the error persists, the encoder wires are probaby disconnected. Tell electrical to recheck the wires\connectors. </

### Suddenly No Resposne 
In this case, stop and restart the ODrive program. The problem is still being investigated \
  
### Unknown ACK and Failure 
In this case, stop and restart the ODrive program. The problem is also still being investigated \

### No ODrive Module
Make sure you are connected to wifi. \
`$ cd ~/.mrover` \
`$ source bin/activate` \
`$ pip3 install odrive`

### Other Errors
Find someone on ESW. Or just go ahead and contact madcowswe himself.

### ToDo 
- [ ] Maybe try to combine into one program instead of running 3 instances

### ToDo - ROS Migration
- [ ] UPDATE THE README - it's very old
- [ ] On README, check if links still work.
- [ ] Figure out how to get the ODrive library to be recognized
- [ ] Test everything
- [ ] Somehow make the code cleaner


### ToDo - CAN Migration
- [ ] MAJOR ISSUE - There is function in CAN equivalent to enable/disable/feed watchdog
- [ ] Make sure to manually change IDs of each axis


### Notes
This README.md is extremely outdated. 
