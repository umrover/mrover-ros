## Table of Contents

[Project Overview](#project-overview) \
[Hardware](#hardware) \
[Top-Level Code](#top-level-code) \
[Configuration](#configuration) \
[Topics - Subscriber](#topics---subscriber) \
[Topics - Publisher](#topics---publisher) \
[Watchdog Timeout Period](#watchdog-timeout-period) \
[Setting Up a New ODrive](#setting-up-a-new-odrive) \
[Changing the USB Permissions](#changing-the-usb-permissions) \
[Calibrating The ODrive](#calibrating-the-odrive) \
[ODrive Errors](#odrive-errors) \
[USB Disconnects](#usb-disconnects) \
[Debugging (on the Jetson)](#debugging-on-the-jetson) \
[Common Errors](#common-errors) \
[TODO](#todo) \
[TODO - ROS Migration](#todo---ros-migration)

---

## Project Overview

The ODrives codebase deals with controlling the ODrive motor controllers to drive the wheels.
For the 2022 Mars Rover Rosie, there are 3 ODrive motor controllers to drive the wheels.
Each ODrive connects to the Jetson via USB. Each ODrive has two axes and each axis is assigned
a wheel. To simplify wiring and code, the axis of each ODrive are either all left or all right.
Also, the wheels that are connected to one singular ODrive are opposite each other (i.e. an ODrive
is assigned either the front, middle, or back wheels).

---

## Top Level Code

#### [`odrive.py`](./odrive.py)

This program runs main for one particular ODrive,
taking a command line argument (either front, middle, or back).
Three threads run simultaneously:
One thread to process incoming commands,
one thread to publish ODrive data,
and one thread to manage the watchdog.

#### ODriveBridge

One ODriveBridge object is created in main to store all the configuration data and
manage the states of the ODrive and various other behavior.

#### States

The State object is kept track of in ODriveBridge.
There are various types of States: ArmedState, DisconnectedState, and ErrorState.
The State may change depending on the current ODriveEvent.

#### Disconnected State

In this state the program is searching for the ODrive by its ID number. Once it has found the ODrive, the state will immediately change to Armed.

#### Armed State

In this state the ODrive will respond to velocity commands until instructed otherwise or errors out.

#### Error State

In this state the ODrive has detected a system error and will reboot, going from Disconnected to Armed immediately.

#### Event 

The ODriveEvent enums dictate the behavior of the changing of States.
There ODriveEvents are the following: When the ODrive is disconnected, trying to arm, or
if there is an error.

#### Modrive

The Modrive object abstracts the behavior of the ODrive.
The ODriveBridge creates the same Modrive object
every time it tries to connect to an ODrive. The Modrive object
has functions that allows it to get encoder data, get state data, 
set velocity, and much more.

---

## Configuration

#### [`odrive.yaml`](../../../config/odrive.yaml)

This configuration file allows you to configure various things.

#### Axis - odrive/axis/

You may choose to configure what axis corresponds to either left or right.
This must be consistent with which axes the left and right wheel are connected to electrically.
This must only be 0 or 1 (and both should not share the same number).

#### ODrive Config - odrive/config/

You may choose to configure the ODrive configuration data such as current limit
and watchdog timeout.

#### ODrive IDs - odrive/ids/

You may choose to configure which ODrive controls which wheel pair by listing its ID.
The ODrive IDs of each ODrive can be found using odrivetool. Follow the steps [here](#getting-the-id).
This must be consistent with which ODrive is connected to which wheel pairs electrically.

#### Multiplier - odrive/multiplier/

You may choose to configure the multiplier for the commands and data of the ODrives.
The incoming velocity commands are between 0 and 1,
and the multiplier turns it into turns per second,
which is what the ODrive library accepts.
The vel estimates from the ODrive library are returned in turns per second,
and the multiplier turns it into meters per second,
which is what the GUI expects.
The magnitudes for the left and right multipliers are the same, but the sign may differ.
The velocity estimate multiplier must be consistent with the actual conversion
of turns per second to meters per second.

---

## Topics - Subscriber

#### Drive Velocity Command
Message: [`DriveVelCmd.msg`](../../../msg/DriveVelCmd.msg) "drive_vel_cmd" \
Publisher: teleop \
Subscriber: odrives

---

## Topics - Publisher

#### Drive Velocity Data
Message: [`DriveVelData.msg`](../../../msg/DriveVelData.msg) "drive_vel_data" \
Publisher: odrives \
Subscriber: gui

#### Drive State Data
Message: [`DriveStateData.msg`](../../../msg/DriveStateData.msg) "drive_state_data" \
Publisher: odrives \
Subscriber: gui

---

## Watchdog Timeout Period

1 second

---

## Setting Up a New ODrive
[Link to the most updated guide as of 2021](https://docs.google.com/document/d/1HmKRBJYMA4qaA--7KrE8OWiwN6Ck05E5CHhxUegaq0A/edit)

---

## Getting the ID

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
In the config/odrive.yaml, look at the line that sets the IDs. 
Depending on which ODrive you replaced, change its ID to the new one. 
Rebuild the program. \

---

## Changing the USB Permissions
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

---

## Calibrating The ODrive 

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

---

## ODrive Errors

If the ODrive throws an error, odrive_bridge will tell it to reboot. The error will be displayed in the terminal and the state will be displayed as ErrorState. Depending on which ODrive this happens on, either the front or back motors will be unresponsive for a minute until the ODrive finishes rebooting. The state will automatically change from DisconnectedState to Armed upon completion.

---

## USB Disconnects 
If the ODrive is disconnected from the USB port, odrive_bridge will print "ODrive has been unplugged" to the terminal, and will change the state to DisconnectedState until the connection is reestablished. At that point the state will automantically change to Armed.

---

## Debugging (on the Jetson)
First make sure that the odrives.py is not running.
In order to see if there are any ODrive errors, ssh into the Jetson \
`$ ssh mrover@10.1.0.1` \
`$ mrover` \
The terminal should now display that you are on the Jetson.
`$ cd ~/.mrover` \
`$ source bin/activate` \
`$ odrivetool` \
`$ dump_errors(odrv0)` \
`$ dump_errors(odrv1)` 

---

## Common Errors

#### ODrive is Not Responding to Calibration 
At this point you should be in odrivetool, if not, follow steps above to get there. \
`$ dump_errors(odrvX, True)` \
`$ dump_errors(odrvX, True)` \
If an `ENCODER_HALL_ERROR` shows up only the first time, you are good to try calibration again. If no errors show up at all,
or if the error persists, re-check your wiring.

#### Unexplained USB Failures
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
 
#### ERROR_ILLEGAL_HALL_STATE
`dump_errors(odrvX, True)` \
If the error persists, the encoder wires are probaby disconnected. Tell electrical to recheck the wires\connectors. </

#### Suddenly No Resposne 
In this case, stop and restart the ODrive program. The problem is still being investigated \
  
#### Unknown ACK and Failure 
In this case, stop and restart the ODrive program. The problem is also still being investigated \

#### No ODrive Module
Make sure you are connected to wifi. \
`$ cd ~/.mrover` \
`$ source bin/activate` \
`$ pip3 install odrive`

#### Other Errors
Find someone on ESW. Or just go ahead and contact madcowswe himself.

---

## TODO
- [ ] _usb_lock.acquire is called so many times... try to find a pattern
- [ ] Look into changing multiplier so config changes from [0, 1] to m/s and there is a different value for turns to meters
- [ ] Code clean up
- [ ] Move beaglebone stuff into config file
- [ ] Test with ROS
- [ ] Make sure messages and topics are consistent with the gui and teleop programs

---

## TODO - ROS Migration
- [ ] See if code builds in ROS
- [ ] See if UART locks work. If they do not, maybe try to revert back to how it worked in 2022 code?
Otherwise, need to deal with exception handling. If that does not work either, then look for another
solution.
