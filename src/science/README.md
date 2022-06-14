Code for the science bridge program that will interpret UART messages from a Nucleo
======================================================================================
### About
Reads and parses NMEA like messages from the onboard 
science nucleo to operate the science box and get relevant data.

### Hardware
- NVIDIA JetsonNX
- STM32G050C8 custom board
- RX/TX connection cables 


### Overview
The science.py program will constantly read in UART messages from the Nucleo 
and it will publish them to certain topics depending on the data received. 
The sciencecomms.py program includes all the commonly shared functions 
and data needed for UART communication between the jetson and the nucleo. 
sciencecomms.py also has information on the configuration of the science, 
including which devices map to which mosfet devices.

In order for the user to control certain science devices 
(such as heaters and various LEDs),
there are services that have been made.
The services make use of sciencecomms.py to transmit
messages over UART.
The services can be located in the scripts folder.

#### Services

**Change Arm Laser State [service]** \
Server: [change_arm_laser_state_server](https://github.com/umrover/mrover-ros/blob/main/scripts/change_arm_laser_state_server.py) \
Client: gui \

**Change Auton LED State [service]** \
Server: [change_auton_led_state_server](https://github.com/umrover/mrover-ros/blob/main/scripts/change_auton_led_state_server.py) \
Client: gui \

**Change Heater Auto Shut Off State [service]** \
Server: [change_heater_auto_shut_off_state](https://github.com/umrover/mrover-ros/blob/main/scripts/change_heater_auto_shut_off_state.py) \
Client: gui \

**Change Heater State [service]** \
Server: [change_heater_state_server](https://github.com/umrover/mrover-ros/blob/main/scripts/change_heater_state_server.py) \
Client: gui \

**Change Servo Angles [service]** \
Server: [change_servo_angles_server](https://github.com/umrover/mrover-ros/blob/main/scripts/change_servo_angles_server.py) \
Client: gui \

**Change UV LED Carousel State [service]** \
Server: [change_uv_led_carousel_server](https://github.com/umrover/mrover-ros/blob/main/scripts/change_uv_led_carousel_server.py) \
Client: gui \

**Change UV LED End Effector State [service]** \
Server: [change_uv_led_end_effector_server](https://github.com/umrover/mrover-ros/blob/main/scripts/change_uv_led_end_effector_server.py) \
Client: gui \

**Change White LED State [service]** \
Server: [change_white_led_server](https://github.com/umrover/mrover-ros/blob/main/scripts/change_white_led_server.py) \
Client: gui \

#### Publishers

**Heater Auto Shut Off Data [publisher]** \
Messages: [Enable.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Enable.msg) "/heater_auto_shut_off_data" \
Publishers: science\
Subscribers: teleop

**Heater State Data [publisher]** \
Messages: [Heater.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Heater.msg) "/heater_state_data" \
Publishers: science\
Subscribers: teleop

**Spectral Data [publisher]** \
Messages: [Spectral.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Spectral.msg) "/spectral_data" \
Publishers: science \
Subscribers: gui

**Spectral Triad Data [publisher]** \
Messages: [Spectral.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Spectral.msg) "/spectral_triad_data" \
Publishers: science \
Subscribers: gui

**Thermistor Data [publisher]** \
Messages: [Thermistor.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Thermistor.msg) "/thermistor_data" \
Publishers: science\
Subscribers: gui

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

**Mosfet Cmd**
- `$MOSFET,<device>,<enable>,<extra padding>`
- Cmd is 30 characters long
- The device represents the mosfet device being activated (0 to 11)

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
