Code for the science bridge program that will interpret UART messages from a Nucleo
======================================================================================
### About
Writes, reads and parses NMEA like messages from the onboard 
science nucleo to operate the science boxes and get relevant data.

### Hardware
- NVIDIA JetsonNX
- STM32G050C8 custom board
- RX/TX connection cables 

### Spectral Sensor Data
Reads the 6 channel data from 3 different spectral sensors in one NMEA style sentence from an STM32 Nucleo Board over UART. 
#### LCM Channels Publishing/Subscribed To
**Spectral Data [publisher]** \
Messages: [SpectralData.lcm](https://github.com/umrover/mrover-workspace/blob/main/rover_msgs/SpectralData.lcm) "/spectral_data" \
Publishers: jetson/science_bridge \
Subscribers: base_station/gui
### UART Message
Format of the data string
- `$SPECTRAL, d0_msb_ch0, d0_lsb_ch0, d0_msb_ch1, d0_lsb_ch1, d0_msb_ch2, d0_lsb_ch2, d0_msb_ch3, d0_lsb_ch3, d0_msb_ch4, d0_lsb_ch4, d0_msb_ch5, d0_lsb_ch5, d1_msb_ch0, d1_lsb_ch0, d1_msb_ch1, d1_lsb_ch1, d1_msb_ch2, d1_lsb_ch2, d1_msb_ch3, d1_lsb_ch3, d1_msb_ch4, d1_lsb_ch4, d1_msb_ch5, d1_lsb_ch5,  d2_msb_ch0, d2_lsb_ch0, d2_msb_ch1, d2_lsb_ch1, d2_msb_ch2, d2_lsb_ch2, d2_msb_ch3, d2_lsb_ch3, d2_msb_ch4, d2_lsb_ch4, d2_msb_ch5, d2_lsb_ch5,`
- String is 155 characters long

### Spectral Triad Data
Reads the 18 channel data from a spectral triad sensor in one NMEA style sentence from an STM32 Nucleo Board over UART. 
#### LCM Channels Publishing/Subscribed To
**Spectral Triad Data [publisher]** \
Messages: [SpectralData.lcm](https://github.com/umrover/mrover-workspace/blob/main/rover_msgs/SpectralData.lcm) "/spectral_triad_data" \
Publishers: jetson/science_bridge \
Subscribers: base_station/gui
### UART Message
Format of the data string
- `$TRIAD, d0_msb_ch0, d0_lsb_ch0, d0_msb_ch1, d0_lsb_ch1, d0_msb_ch2, d0_lsb_ch2, d0_msb_ch3, d0_lsb_ch3, d0_msb_ch4, d0_lsb_ch4, d0_msb_ch5, d0_lsb_ch5, d1_msb_ch0, d1_lsb_ch0, d1_msb_ch1, d1_lsb_ch1, d1_msb_ch2, d1_lsb_ch2, d1_msb_ch3, d1_lsb_ch3, d1_msb_ch4, d1_lsb_ch4, d1_msb_ch5, d1_lsb_ch5,  d2_msb_ch0, d2_lsb_ch0, d2_msb_ch1, d2_lsb_ch1, d2_msb_ch2, d2_lsb_ch2, d2_msb_ch3, d2_lsb_ch3, d2_msb_ch4, d2_lsb_ch4, d2_msb_ch5, d2_lsb_ch5,`
- String is 155 characters long

### Thermistor Data
Reads three seperate temperatures from one NMEA-style sentence from the nucleo over UART.
#### LCM Channels Publishing/Subscribed To
**Thermistor Data [Publisher]** \
Messages: [ThermistorData.lcm](https://github.com/umrover/mrover-workspace/blob/main/rover_msgs/ThermistorData.lcm) "/thermistor_data" \
Publishers: jetson/science_bridge\
Subscribers: jetson/teleop
#### UART Message
Format of the data string
- `$THERMISTOR,<temp0>,<temp1>,<temp2>,<extra padding>`
- String is 50 characters long

### Mosfet Cmd
Writes an NMEA like message over UART to the Nucleo in order to turn a specified mosfet device on or off. \
Controls the auton LEDs, robotic arm laser, UV LEDs, UV Bulb, nichrome wires, and raman lasers.
#### LCM Channels Publishing/Subscribed To
**Mosfet Command [subscriber]** \
Messages: [MosfetCmd.lcm](https://github.com/umrover/mrover-workspace/blob/main/rover_msgs/MosfetCmd.lcm) "/mosfet_cmd" \
Publishers: base_station/gui \
Subscribers: jetson/science_bridge

**Nav Status [subscriber]** \
Messages: [NavStatus.lcm](https://github.com/umrover/mrover-workspace/blob/main/rover_msgs/NavStatus.lcm) "/nav_status" \
Publishers: jetson/nav \
Subscribers: jetson/science_bridge

#### UART Message
Format of the UART NMEA command
- `$MOSFET,<device>,<enable>,<extra padding>`
- String is 30 characters long
- The device represents the mosfet device being activated (0 to 11)

### Servo Cmd
Writes NMEA like messages over UART to the Nucleo in order to move servo to specific angles (in degrees). 
#### LCM Channels Publishing/Subscribed To 
**Servo Command [subscriber]** \
Messsages: [ServoCmd.lcm](https://github.com/umrover/mrover-workspace/blob/main/rover_msgs/ServoCmd.lcm) "/servo_cmd" \
Publishers: base_station/gui \
Subscribers: jetson/science_bridge
#### UART Message
Format of the UART NMEA command
- `$SERVO,<angle0>,<angle1>,<angle2>,<extra padding>`
- String is 30 characters long
- The angles are in degrees


### Heater Cmd
Writes NMEA like messages over UART to the Nucleo in order to turn a specific heater on. 
#### LCM Channels Publishing/Subscribed To 
**Heater Command [subscriber]** \
Messsages: [Heater.lcm](https://github.com/umrover/mrover-workspace/blob/main/rover_msgs/Heater.lcm) "/heater_cmd" \
Publishers: base_station/gui \
Subscribers: jetson/science_bridge
#### UART Message
Format of the UART NMEA command
- `$MOSFET,<device>,<enable>,<extra padding>`
- String is 30 characters long
- The device represents the mosfet device being activated
- Heaters are mosfet devices 5, 6, and 7


### Heater State Data
Reads data on whether or not the heater is on.
#### LCM Channels Publishing/Subscribed To
**Heater State Data [Publisher]** \
Messages: [Heater.lcm](https://github.com/umrover/mrover-workspace/blob/main/rover_msgs/Heater.lcm) "/heater_state_data" \
Publishers: jetson/science_bridge\
Subscribers: jetson/teleop
#### UART Message
Format of the data string
- `$HEATER,<device>,<enable>,<extra padding>`
- String is 50 characters long


### Heater Auto Shutdown Cmd
Writes NMEA like messages over UART to the Nucleo in order to turn the auto shutdown feature for heater on or off. 
#### LCM Channels Publishing/Subscribed To 
**Heater Command [subscriber]** \
Messsages: [HeaterAutoShutdown.lcm](https://github.com/umrover/mrover-workspace/blob/main/rover_msgs/HeaterAutoShutdown.lcm) "/heater_auto_shutdown_cmd" \
Publishers: base_station/gui \
Subscribers: jetson/science_bridge
#### UART Message
Format of the UART NMEA command
- `$AUTOSHUTOFF,<enable>,<extra padding>`
- String is 30 characters long
- The enable represents whether or not auto shutoff should be enabled or not

### Heater Auto Shutdown Data
Reads data on whether or not the heater auto shutdown feature is on.
#### LCM Channels Publishing/Subscribed To
**Heater Auto Shutdown Data [Publisher]** \
Messages: [HeaterAutoShutdown.lcm](https://github.com/umrover/mrover-workspace/blob/main/rover_msgs/HeaterAutoShutdown.lcm) "/heater_auto_shutdown_data" \
Publishers: jetson/science_bridge\
Subscribers: jetson/teleop
#### UART Message
Format of the data string
- `$AUTOSHUTOFF,<device>,<enable>,<extra padding>`
- String is 30 characters long


## TODO
- [ ] Code clean up
- [ ] Move beaglebone stuff into config file
- [ ] Don't need to wait for all tags to be seen
- [ ] Test with ROS
