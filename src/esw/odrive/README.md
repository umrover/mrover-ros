## Table of Contents

[Project Overview](#project-overview) \
[Hardware](#hardware) \
[Top-Level Code](#top-level-code) \
[Configuration](#configuration) \
[Topics - Subscriber](#topics---subscriber) \
[Topics - Publisher](#topics---publisher) \
[Watchdog Timeout Period](#watchdog-timeout-period) \
[Setting up a New ODrive](#setting-up-a-new-odrive) \
[Getting the ID](#getting-the-id) \
[Changing the USB Permissions](#changing-the-usb-permissions) \
[Calibrating the ODrive](#calibrating-the-odrive) \
[Testing the Motor using odrivetool](#testing-the-motor-using-odrivetool) \
[Tuning Control Loops](#tuning-control-loops) \
[Program Behavior if ODrive Errors](#program-behavior-if-odrive-errors) \
[Program Behavior if USB Disconnects](#program-behavior-if-usb-disconnects) \ 
[Checking if There are Errors](#checking-if-there-are-errors) \
[Common Errors](#common-errors) \
[TODO](#todo) \
[TODO - ROS Migration](#todo---ros-migration)

---

## Project Overview

The ODrive codebase deals with controlling the ODrive motor controllers to
drive the wheels. For the 2022 Mars Rover Rosie, there are 3 ODrive motor
controllers to drive the wheels. Each ODrive connects to the Jetson via USB.
Each ODrive has two axes and each axis is assigned a wheel. To simplify wiring
and code, the axis of each ODrive are either all left or all right. Also, the
wheels that are connected to one singular ODrive are opposite each other (i.e.
an ODrive is assigned either the front, middle, or back wheels).

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

Dictates the behavior of the changing of States.
The ODriveEvents are the following: When the ODrive is disconnected,
trying to arm, or if there is an error.

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

#### Ratio - odrive/ratio/

You may choose to configure the ratios of the wheels for the ODrives.
By changing the ratios, this will affect the magnitude of what the raw speed [0, 1] means
from incoming VelCmdData messages and also affect the data received. This is desired because the ODrives
keep track of everything in terms of turns.
The way to change the ratio config is the following: If one turn is equal to 40 meters for the right wheel, then the meters_to_turns_ratio_right variable should be set to 40.
The ratios must be consistent with the actual conversions between incoming commands, turns, and meters.

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

## Setting up a New ODrive

Guide for the Nanotec/Maxon motors

1. Open up odrivetool. On the Jetson run `source ~/.mrover/build_env/bin/activate` and type `odrivetool`.
This should open up and say we connected to ODrive. We want (I think) version 0.5.4.
2. If it's an ODrive we used before, just double check these parameters. Otherwise, set the following anyway (numbers in parentheses were used for Rosie '22): 
- `odrvX.axisY.motor.config.pole_pairs` which depends on the motor and is on the data sheet (was 7),
- `odrvX.axisY.motor.config.resstance_calib_max_voltage` which should be either 1.2 or 0.6 (was 0.6),
- `odrvX.axisY.motor.config.requested_current_range` which should probably be 8.0 (was 8.0),
- `odrvX.axisY.motor.config.torque_constant` which it should say on the motor's data sheet and the units are Nm per amp and it might be 52.5/1000 (was 52.5/1000)
- `odrvX.axisY.encoder.config.cpr` which should be 6 x pole_pairs (was 42)
- `odrvX.axisY.controller.config.pos_gain` which should be 0.01 (was 0.01)
- `odrvX.axisY.controller.config.vel_gain` which should be 0.01 (was 0.01)
- `odrvX.axisY.controller.config.vel_integrator_gain` which should be 0 (was 0)
- `odrvX.axisY.controller.config.vel_limit` which is speed and it could be 1000 or so unit is turns/second and we once wanted 5300 rpm (was 1000)
- `odrvX.axisY.motor.config.calibration_current` which should be 1.45 or 0.7 (was 0.7). 
3. Save configuration and reboot by running `odrvX.save_configuration()`. If this returns false, make sure all axes are in the IDLE state by doing `odrvX.axisY.requested_state = AXIS_STATE_IDLE`. Then reboot by running `odrvX.reboot`.

---

## Getting the ID

Each ODrive has a unique serial ID. In order to determine that this ID is, follow the steps on 
[this](https://docs.odriverobotics.com/#downloading-and-installing-tools) website on the hoverboard guide page. To activate odrivetool
 follow the steps below. You can do this on the base station or the Jetson. On the Jetson make sure the \
 odrive_bridge program on the Jetson is [deactivated](Debugging). \
Open up odrivetool. On the Jetson run `source ~/.mrover/build_env/bin/activate` and type `odrivetool`.
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

## Calibrating the ODrive 

Calibration must be done manually. Once again ssh into the Jetson and go to the .mrover folder 
and start running odrivetool. \
Run `source ~/.mrover/build_env/bin/activate` and type `odrivetool`.
The odrives should automatically connect. Using the odrvX.axisY (0 or 1 depending on the ODrive with the id) in place of m_axis, execute all of the following commands for axis0 and axis1. \

`$ odrvX.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE ` \
The motor should beep and start calibrating now.

You can check to see if it completed successfully by running `dump_errors(odrvX)`.
If there are errors, you can try doing `dump_errors(odrvX, True)` to first dump the errors,
and then you can retry the calibration sequence.

If there are still errors, you may need to check the **Errors** section below. 
Once it has finished, type \
`$ odrvX.axis0.motor.config.pre_calibrated = True ` \
`$ odrvX.axis0.encoder.config.pre_calibrated = True ` \
Repeat these three commands for axis1 as well. Then type \
`$ odrvX.reboot()` 

---

## Testing the Motor Using odrivetool

1. Open up odrivetool. On the Jetson run `source ~/.mrover/build_env/bin/activate` and type `odrivetool`.
2. Put the odrive in closed loop by doing `odrvX.axisY.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL`
3. Set the velocity by doing `odrvX.axisY.controller.input_vel = [insert velocity]` where velocity is in turns per second.
4. To get the actual speed it's going at, run `odrvX.axisY.encoder.vel_estimate` where it returns a number in turns per second.

---

## Tuning Control Loops

1. For velocity control you want to plot `odrvX.axisY.encoder.vel_estimate` and `odrvX.axisY.controller.input_vel`.
2. Type in `start_liveplotter(lambda:[var1, var])` to get a graph.
3. If it's saying from matplotlib_image does not exist, then in the odrive tool terminal, run `import matplotlib` and `matplotlib.use('GTKAgg')`
4. Tune the control loops as you please, hopefully you are following a forum and/or know what you're doing (do not kill the ODrives/motors please!)

---

## Program Behavior if ODrive Errors

If the ODrive throws an error, odrive_bridge will tell it to reboot. The error will be displayed in the terminal and the state will be displayed as ErrorState. Depending on which ODrive this happens on, either the front or back motors will be unresponsive for a minute until the ODrive finishes rebooting. The state will automatically change from DisconnectedState to Armed upon completion.

---

## Program Behavior if USB Disconnects 
If the ODrive is disconnected from the USB port, odrive_bridge will print "ODrive has been unplugged" to the terminal, and will change the state to DisconnectedState until the connection is reestablished. At that point the state will automantically change to Armed.

---

## Checking if There are Errors
First make sure that the odrives.py is not running.
Open up odrivetool. On the Jetson run `source ~/.mrover/build_env/bin/activate` and type `odrivetool`.
`$ dump_errors(odrv0)` \
`$ dump_errors(odrv1)` 

---

## Common Errors

#### ODrive Is Not Responding To Calibration 
Open up odrivetool. On the Jetson run `source ~/.mrover/build_env/bin/activate` and type `odrivetool`.
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

#### USB Forwarding on VM 

Make sure the ODrive is connected via USB and type \
`$ lsusb` . From list find the idVendor, idProduct, and MODE of the odrive. It will be listed under the info for the InterBiometrics 
device. \
`$ sudo vi /etc/udev/rules.d/99-usb-serial.rules` \
`$ SUBSYSTEMS=="usb", ATTRS{idVendor}=="[__idVendor__]", ATTRS{idProduct}=="[__idProduct__]", GROUP="mrover", MODE="[__MODE__]" ` \
Restart the VM.
 
#### ERROR_ILLEGAL_HALL_STATE
`dump_errors(odrvX, True)` \
If the error persists, the encoder wires are probaby disconnected. Tell electrical to recheck the wires\connectors. </

#### Suddenly No Response
In this case, stop and restart the ODrive program. The problem is still being investigated \
  
#### Unknown ACK and Failure 
In this case, stop and restart the ODrive program. The problem is also still being investigated \

#### No ODrive Module
Make sure you are connected to wifi. \
On the Jetson run `source ~/.mrover/build_env/bin/activate`.
`$ pip3 install odrive`

#### Other Errors
Find someone on ESW. Or just go ahead and contact madcowswe himself.

---

## TODO
- [ ] SOMEONE DOUBLE CHECK IF THE meters_to_turns_ratio IS CORRECT... IT IS LIKELY WRONG.
I ADDED ASSERTS IN CASE I MESSED UP SOMEWHERE.
- [ ] Test with ROS
- [ ] Make sure messages and topics are consistent with the gui and teleop programs

---

## TODO - ROS Migration
- [ ] See if code builds in ROS
- [ ] See if UART locks work. If they do not, maybe try to revert back to how it worked in 2022 code?
Otherwise, need to deal with exception handling. If that does not work either, then look for another
solution.
