Code for the custom science Nucleo board to transmit and receive UART messages 
======================================================================================
### About
Reads and parses NMEA like messages from the onboard 
custom science Nucleo board to do various tasks such as
operate the science box and get relevant data and also
control the arm laser and auton LED array.

### Hardware
- NVIDIA JetsonNX
- STM32G050C8 custom board
- RX/TX connection cables 


### Overview
The science_board.py program will constantly read in UART messages from the Nucleo 
and it will publish them to certain topics depending on the data received. 
This program includes all the functions 
and data needed for UART communication between the Jetson and the Nucleo. 
It also has information on the configuration of the science, 
including which devices map to which MOSFET devices.

In order for the user to control certain science devices 
(such as heaters and various LEDs),
there are services that have been made.
The servers for these services run in science_board.py.
The servers will transmit
messages over UART if a client tries to talk with it.

Locks exist to prevent two functions from trying to access the UART line at the same time

### Services - Server

**Change Arm Laser State [Server]** \
Server: [ChangeDeviceState](https://github.com/umrover/mrover-ros/blob/main/srv/ChangeDeviceState.srv) "change_arm_laser_state" \
Client: gui \

**Change Auton LED State [Server]** \
Server: [ChangeAutonLEDState](https://github.com/umrover/mrover-ros/blob/main/srv/ChangeAutonLEDState.srv)  "change_auton_led_state" \
Client: teleop \

**Change Heater Auto Shut Off State [Server]** \
Server: [change_heater_auto_shut_off_state](https://github.com/umrover/mrover-ros/blob/main/srv/ChangeDeviceState.srv)  "change_heater_auto_shut_off" \
Client: gui \

**Change Heater State [Server]** \
Server: [ChangeHeaterState](https://github.com/umrover/mrover-ros/blob/main/srv/ChangeHeaterState.srv)  "change_heater_state" \
Client: gui \

**Change Servo Angles [Server]** \
Server: [change_servo_angles_server](https://github.com/umrover/mrover-ros/blob/main/srv/ChangeDeviceState.srv)  "change_servo_angles" \
Client: gui \

**Change UV LED Carousel State [Server]** \
Server: [change_uv_led_carousel_server](https://github.com/umrover/mrover-ros/blob/main/srv/ChangeDeviceState.srv) ) "change_uv_led_carousel" \
Client: gui \

**Change UV LED End Effector State [Server]** \
Server: [change_uv_led_end_effector_server](https://github.com/umrover/mrover-ros/blob/main/srv/ChangeDeviceState.srv)  "change_uv_led_end_effector" \
Client: gui \

**Change White LED State [Server]** \
Server: [change_white_led_server](https://github.com/umrover/mrover-ros/blob/main/srv/ChangeDeviceState.srv)  "change_white_led" \
Client: gui \

### Topics - Publisher

**Heater Auto Shut Off Data [Publisher]** \
Messages: [Enable.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Enable.msg) "heater_auto_shut_off_data" \
Publisher: science_board \
Subscriber: gui

**Heater State Data [Publisher]** \
Messages: [Heater.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Heater.msg) "heater_state_data" \
Publisher: science_board \
Subscriber: gui

**Spectral Data [Publisher]** \
Messages: [Spectral.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Spectral.msg) "spectral_data" \
Publisher: science_board \
Subscriber: gui

**Spectral Triad Data [Publisher]** \
Messages: [Spectral.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Spectral.msg) "spectral_triad_data" \
Publisher: science_board \
Subscriber: gui

**Thermistor Data [Publisher]** \
Messages: [Thermistor.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Thermistor.msg) "thermistor_data" \
Publisher: science_board \
Subscriber: gui

### UART Messages

**Heater Auto Shut Off Cmd/Data**
Format of the UART NMEA command
- `$AUTOSHUTOFF,<enable>,<extra padding>`
- Cmd and data are 30 characters long
- Enable is state of auto shut off feature

**Heater State Data Cmd/Data**
- `$HEATER,<device>,<enable>,<extra padding>`
- Cmd and data are 30 characters long
- Enable is state of heater

**MOSFET Cmd**
- `$MOSFET,<device>,<enable>,<extra padding>`
- Cmd is 30 characters long
- The device represents the MOSFET device being activated (0 to 11)

**Spectral Data**
- `$SPECTRAL, d0_msb_ch0, d0_lsb_ch0, d0_msb_ch1, d0_lsb_ch1, d0_msb_ch2, d0_lsb_ch2, d0_msb_ch3, d0_lsb_ch3, d0_msb_ch4, d0_lsb_ch4, d0_msb_ch5, d0_lsb_ch5, d1_msb_ch0, d1_lsb_ch0, d1_msb_ch1, d1_lsb_ch1, d1_msb_ch2, d1_lsb_ch2, d1_msb_ch3, d1_lsb_ch3, d1_msb_ch4, d1_lsb_ch4, d1_msb_ch5, d1_lsb_ch5,  d2_msb_ch0, d2_lsb_ch0, d2_msb_ch1, d2_lsb_ch1, d2_msb_ch2, d2_lsb_ch2, d2_msb_ch3, d2_lsb_ch3, d2_msb_ch4, d2_lsb_ch4, d2_msb_ch5, d2_lsb_ch5,<extra padding>`
- Data is 155 characters long
- 6 channel data from each of the three spectrals

**Servo Cmd**
- `$SERVO,<angle0>,<angle1>,<angle2>,<extra padding>`
- Cmd is 155 characters long
- The angles are in degrees

**Thermistor Data**
- `$THERMISTOR,<temp0>,<temp1>,<temp2>,<extra padding>`
- Data is 155 characters long
- Temperature is in celsius

**Triad Data**
- `$TRIAD, d0_msb_ch0, d0_lsb_ch0, d0_msb_ch1, d0_lsb_ch1, d0_msb_ch2, d0_lsb_ch2, d0_msb_ch3, d0_lsb_ch3, d0_msb_ch4, d0_lsb_ch4, d0_msb_ch5, d0_lsb_ch5, d1_msb_ch0, d1_lsb_ch0, d1_msb_ch1, d1_lsb_ch1, d1_msb_ch2, d1_lsb_ch2, d1_msb_ch3, d1_lsb_ch3, d1_msb_ch4, d1_lsb_ch4, d1_msb_ch5, d1_lsb_ch5,  d2_msb_ch0, d2_lsb_ch0, d2_msb_ch1, d2_lsb_ch1, d2_msb_ch2, d2_lsb_ch2, d2_msb_ch3, d2_lsb_ch3, d2_msb_ch4, d2_lsb_ch4, d2_msb_ch5, d2_lsb_ch5,<extra padding>`
- Data is 158 characters long
- 18 channel data


## TODO
- [ ] Code clean up
- [ ] Move beaglebone stuff into config file
- [ ] Test with ROS
- [ ] Make sure messages and topics are consistent with the gui and teleop programs
- [ ] Move self.MOSFET_DEV_MAP and self.ser into a config file

## TODO - ROS Migration
- [ ] See if code builds in ROS
- [ ] Check to see if we want to keep in scripts folder or src folder 
(it's probably the src folder but this isn't a big change)
- [ ] See if we need to include the files in some sort of launch file?
- [ ] See if UART locks work. If they do not, maybe try to revert back to how it worked in 2022 code?
Otherwise, need to deal with exception handling. If that does not work either, then look for another
solution.
