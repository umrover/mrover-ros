## Table of Contents

[Project Overview](#project-overview) \
[Hardware](#hardware) \
[Top-Level Code](#top-level-code) \
[Configuration](#configuration) \
[Services - Server](#services---server) \
[Topics - Publisher](#topics---publisher) \
[UART Messages](#uart-messages) \
[TODO](#todo) \
[TODO - ROS Migration](#todo---ros-migration)

---

## Project Overview
The science_board codebase deals with reading and parsing NMEA like messages from the STM32 chip on the
science PCB to complete tasks for almost every mission. These tasks
include operating the science box and getting relevant data during the Science task,
controlling the arm laser dring the Equipment Servicing task, and controlling
the LED array used during the Autonomous Traversal task.

---

## Hardware
- NVIDIA JetsonNX
- PCB with a STM32G050C8 chip
- RX/TX connection cables 

---

## Top Level Code

#### [`science_board.py`](./science_board.py)

This program runs main.
This reads NMEA like messages via UART from the STM32 chip and depending on the
data received, it will publish the data to ROS topics.

In order for the user to control devices such as heaters and various LEDs,
this program acts as the server for various services. Upon receiving
a service request, the program will send an NMEA like message via UART to the
STM32 chip to issue out the command.

Locks exist to prevent two functions from trying to access the UART line at the same time

## Configuration

#### [`science_board.yaml`](../../../config/science_board.yaml)

This configuration file allows you to configure various things.

#### LED Array to UART Mapping - science_board/led_map/

You may choose to configure what number to pack
into the UART message in order to get the particular color to appear on the
LED Array. This must be consistent with the firmware flashed on the STM32 chip.

#### Device to MOSFET Device Mapping - science_board/mosfet_device_map/

You may choose to configure which device maps to which MOSFET device.
This must be consistent to which MOSFET device the device is connected to electrically.

#### UART Serial Info - science_board/serial/

You may choose to configure the UART serial info, such as baudrate and timeout.
The baudrate must be consistent with the firmware flased on the STM32 chip.

#### Miscellaneous Info - science_board/info/

You may choose to configure settings such as the maximum amount of errors permitted,
the sleep duration, and the length of every UART message transmitted.
The UART transmit message length must be consistent with the firmware flashed on the
STM32 chip.

## Services - Server

#### Change Arm Laser State
Service: [`ChangeDeviceState.srv`](../../../srv/ChangeDeviceState.srv) "change_arm_laser_state" \
Server: science_board \
Client: gui \

#### Change Auton LED State
Service: [`ChangeAutonLEDState.srv`](../../../srv/ChangeAutonLEDState.srv) "change_auton_led_state" \
Server: science_board \
Client: teleop \

#### Change Heater Auto Shut Off State
Service: [`ChangeDeviceState.srv`](../../../srv/ChangeDeviceState.srv) "change_heater_auto_shut_off" \
Server: science_board \
Client: gui \

#### Change Heater State
Service: [`ChangeHeaterState.srv`](../../../srv/ChangeHeaterState.srv) "change_heater_state" \
Server: science_board \
Client: gui \

#### Change Servo Angles
Service: [`ChangeDeviceState.srv`](../../../srv/ChangeDeviceState.srv) "change_servo_angles" \
Server: science_board \
Client: gui \

#### Change UV LED Carousel State
Service: [`ChangeDeviceState.srv`](../../../srv/ChangeDeviceState.srv) "change_uv_led_carousel" \
Server: science_board \
Client: gui \

#### Change UV LED End Effector State
Service: [`ChangeDeviceState.srv`](../../../srv/ChangeDeviceState.srv) "change_uv_led_end_effector" \
Server: science_board \
Client: gui \

#### Change White LED State
Service: [`ChangeDeviceState.srv`](../../../srv/ChangeDeviceState.srv) "change_white_led" \
Server: science_board \
Client: gui \

## Topics - Publisher

#### Heater Auto Shut Off Data
Message: [`Enable.msg`](../../../msg/Enable.msg) "heater_auto_shut_off_data" \
Publisher: science_board \
Subscriber: gui

#### Heater State Data
Message: [`Heater.msg`](../../../msg/Heater.msg) "heater_state_data" \
Publisher: science_board \
Subscriber: gui

#### Spectral Data
Message: [`Spectral.msg`](../../../msg/Spectral.msg) "spectral_data" \
Publisher: science_board \
Subscriber: gui

#### Spectral Triad Data
Message: [`Spectral.msg`](../../../msg/Spectral.msg) "spectral_triad_data" \
Publisher: science_board \
Subscriber: gui

#### Thermistor Data
Message: [`Thermistor.msg`](../../../msg/Thermistor.msg) "thermistor_data" \
Publisher: science_board \
Subscriber: gui

## UART Messages

#### Heater Auto Shut Off Cmd/Data
- `$AUTOSHUTOFF,<enable>,<extra padding>`
- Cmd and data are 30 characters long
- Enable is state of auto shut off feature

#### Heater State Data Cmd/Data
- `$HEATER,<device>,<enable>,<extra padding>`
- Cmd and data are 30 characters long
- Enable is state of heater

#### MOSFET Cmd
- `$MOSFET,<device>,<enable>,<extra padding>`
- Cmd is 30 characters long
- The device represents the MOSFET device being activated (0 to 11)

#### Spectral Data
- `$SPECTRAL, d0_msb_ch0, d0_lsb_ch0, d0_msb_ch1, d0_lsb_ch1, d0_msb_ch2, d0_lsb_ch2, d0_msb_ch3, d0_lsb_ch3, d0_msb_ch4, d0_lsb_ch4, d0_msb_ch5, d0_lsb_ch5, d1_msb_ch0, d1_lsb_ch0, d1_msb_ch1, d1_lsb_ch1, d1_msb_ch2, d1_lsb_ch2, d1_msb_ch3, d1_lsb_ch3, d1_msb_ch4, d1_lsb_ch4, d1_msb_ch5, d1_lsb_ch5,  d2_msb_ch0, d2_lsb_ch0, d2_msb_ch1, d2_lsb_ch1, d2_msb_ch2, d2_lsb_ch2, d2_msb_ch3, d2_lsb_ch3, d2_msb_ch4, d2_lsb_ch4, d2_msb_ch5, d2_lsb_ch5,<extra padding>`
- Data is 155 characters long
- 6 channel data from each of the three spectrals

#### Servo Cmd
- `$SERVO,<angle0>,<angle1>,<angle2>,<extra padding>`
- Cmd is 155 characters long
- The angles are in degrees

#### Thermistor Data
- `$THERMISTOR,<temp0>,<temp1>,<temp2>,<extra padding>`
- Data is 155 characters long
- Temperature is in celsius

#### Triad Data
- `$TRIAD, d0_msb_ch0, d0_lsb_ch0, d0_msb_ch1, d0_lsb_ch1, d0_msb_ch2, d0_lsb_ch2, d0_msb_ch3, d0_lsb_ch3, d0_msb_ch4, d0_lsb_ch4, d0_msb_ch5, d0_lsb_ch5, d1_msb_ch0, d1_lsb_ch0, d1_msb_ch1, d1_lsb_ch1, d1_msb_ch2, d1_lsb_ch2, d1_msb_ch3, d1_lsb_ch3, d1_msb_ch4, d1_lsb_ch4, d1_msb_ch5, d1_lsb_ch5,  d2_msb_ch0, d2_lsb_ch0, d2_msb_ch1, d2_lsb_ch1, d2_msb_ch2, d2_lsb_ch2, d2_msb_ch3, d2_lsb_ch3, d2_msb_ch4, d2_lsb_ch4, d2_msb_ch5, d2_lsb_ch5,<extra padding>`
- Data is 158 characters long
- 18 channel data


## TODO
- [ ] Code clean up
- [ ] Move beaglebone stuff into config file
- [ ] Test with ROS
- [ ] Make sure messages and topics are consistent with the gui and teleop programs

## TODO - ROS Migration
- [ ] See if code builds in ROS
- [ ] See if UART locks work. If they do not, maybe try to revert back to how it worked in 2022 code?
Otherwise, need to deal with exception handling. If that does not work either, then look for another
solution.
