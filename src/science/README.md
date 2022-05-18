Code for the science bridge program that will interpret UART messages from a Nucleo
======================================================================================
### About
Writes, reads and parses NMEA like messages from the onboard 
science nucleo to operate the science box and get relevant data.

### Hardware
- NVIDIA JetsonNX
- STM32G050C8 custom board
- RX/TX connection cables 

#### Subscribers

**Arm Laser Command [subscriber]** \
Messages: [Enable.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Enable.msg) "/arm_laser_cmd" \
Publishers: gui \
Subscribers: science

**Heater Auto Shut Off Command [subscriber]** \
Messsages: [Enable.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Enable.msg) "/heater_auto_shut_off_cmd" \
Publishers: gui \
Subscribers: science

**Heater Command [subscriber]** \
Messsages: [Heater.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Heater.msg) "/heater_cmd" \
Publishers: gui \
Subscribers: science

**Raman Laser Command [subscriber]** \
Messages: [Enable.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Enable.msg) "/raman_laser_cmd" \
Publishers: gui \
Subscribers: science

**Servo Command [subscriber]** \
Messsages: [Servo.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Servo.msg) "/servo_cmd" \
Publishers: gui \
Subscribers: science

**UV Bulb Command [subscriber]** \
Messages: [Enable.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Enable.msg) "/uv_bulb_cmd" \
Publishers: gui \
Subscribers: science

**UV LED Command [subscriber]** \
Messages: [Enable.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Enable.msg) "/uv_led_cmd" \
Publishers: gui \
Subscribers: science

**White LED Command [subscriber]** \
Messages: [Enable.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Enable.msg) "/white_led_cmd" \
Publishers: gui \
Subscribers: science

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
- [ ] Don't need to wait for all tags to be seen
- [ ] Test with ROS
