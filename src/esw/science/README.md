## Table of Contents

[Project Overview](#project-overview) \
[System Overview](#system-overview) \
[Top-Level Code](#top-level-code) \
[Configuration](#configuration) \
[Services - Server](#services---server) \
[Topics - Publisher](#topics---publisher) \
[UART Messages](#uart-messages) \
[TODO](#todo) \
[TODO - ROS Migration](#todo---ros-migration)

---

## Project Overview

The science codebase deals with reading and parsing NMEA like messages
from the STM32 chip on the science PCB over UART to complete tasks for almost
every mission. These tasks include operating the science box and getting
relevant data during the Science task, controlling the arm laser during the
Equipment Servicing task, and controlling the LED array used during the
Autonomous Traversal task.

---

## System Overview

Almost all of the onboard science system is controlled by the science board.
The science system can be described as follows: On the rover is a carousel that houses three sites that have instruments to test soil samples. Soil is deposited on the top of carousel to one of the sites, where the soil travels through funnels to beakers that test for different things.

Each site has their own thermistor, heater, spectral sensor, and servos motors. The thermistor and heater are used for the ninhydrin test. The servo motors are used for the strip test to test for ammonia, nitrates, and nitrites as well as pH. The servo motors dip the strips into the solution containing the soil.  The spectral sensor is used for the autofluorescence test which tests for chlorophyll.
A shared UV LED is used inside the carousel for all the sites, as well as a white LED to aid with camera viewing. There is also another UV LED used for disinfecting the scoop after soil acquisitions.

Because many of these functions require just the presence or absence of a voltage difference, we use MOSFETs for many of our devices. The high level view of the MOSFET is that our science board sets the voltage of a particular MOSFET high and the corresponding device turns on.

Our science board has an STM32 chip which we write code for that controls all the logic flow for controlling the MOSFET, reading in spectral data, reading in thermistor data, and controlling the servo sensors. Here, the MOSFET is controlled via output pins from the science board.

The three spectral sensors are close to identical. The communication protocol between the science board and the spectral sensors is I2C, which requires each of the slave devices to have different addresses. Thus, we use an I2C mux.
The thermistor data is read in through the STM32 chip's ADC input pins.
The servo sensors are commanded via PWM (where a certain PWM corresponds to a certain angle).

The Jetson is the one that runs the program to connect the science board system to the ROS network. A script runs on the Jetson to run all the logic to handle this connection.
The Jetson is connected to the STM32 chip via USART. NMEA-like messages are used to transmit information via USART.

---

## Top Level Code

#### [`science.py`](./science.py)

This program runs main.
This reads NMEA like messages via UART from the STM32 chip and depending on the
data received, it will publish the data to ROS topics.

In order for the user to control devices such as heaters and various LEDs,
this program acts as the server for various services. Upon receiving
a service request, the program will send an NMEA like message via UART to the
STM32 chip to issue out the command.

Locks exist to prevent two functions from trying to access the UART line at the same time

---

## Configuration

#### [`science.yaml`](../../../config/science.yaml)

This configuration file allows you to configure various things.

#### LED Array to UART Mapping - science/led_to_id/

You may choose to configure what number to pack
into the UART message in order to get the particular color to appear on the
LED Array. This must be consistent with the firmware flashed on the STM32 chip.

#### Device to MOSFET Number Mapping - science/device_mosfet_numbers/

You may choose to configure which device maps to which MOSFET device number.
This must be consistent to which MOSFET device number the device is connected to electrically.

#### UART Serial Info - science/serial/

You may choose to configure the UART serial info, such as baud rate and timeout.
The baud rate must be consistent with the firmware flased on the STM32 chip.

#### Miscellaneous Info - science/info/

You may choose to configure settings such as
the sleep duration and the length of every UART message transmitted.
The UART transmit message length must be consistent with the firmware flashed on the
STM32 chip.

---

## Services - Server

#### Change Arm Laser State
Service: [`ChangeDeviceState.srv`](../../../srv/ChangeDeviceState.srv) "science/change_arm_laser_state" \
Server: science \
Client: gui \

#### Change Auton LED State
Service: [`ChangeAutonLEDState.srv`](../../../srv/ChangeAutonLEDState.srv) "change_auton_led_state" \
Server: science \
Client: teleop \

#### Change Heater Auto Shut Off State
Service: [`ChangeDeviceState.srv`](../../../srv/ChangeDeviceState.srv) "change_heater_auto_shutoff" \
Server: science \
Client: gui \

#### Change Heater State
Service: [`ChangeHeaterState.srv`](../../../srv/ChangeHeaterState.srv) "change_heater_state" \
Server: science \
Client: gui \

#### Change Servo Angles
Service: [`ChangeDeviceState.srv`](../../../srv/ChangeDeviceState.srv) "change_servo_angles" \
Server: science \
Client: gui \

#### Change UV LED Carousel State
Service: [`ChangeDeviceState.srv`](../../../srv/ChangeDeviceState.srv) "change_uv_led_carousel" \
Server: science \
Client: gui \

#### Change UV LED End Effector State
Service: [`ChangeDeviceState.srv`](../../../srv/ChangeDeviceState.srv) "change_uv_led_end_effector" \
Server: science \
Client: gui \

#### Change White LED State
Service: [`ChangeDeviceState.srv`](../../../srv/ChangeDeviceState.srv) "change_white_led" \
Server: science \
Client: gui \

---

## Topics - Publisher

#### Heater Auto Shut Off Data
Message: [`Enable.msg`](../../../msg/Enable.msg) "science/heater_auto_shutoff_data" \
Publisher: science \
Subscriber: gui

#### Heater State Data
Message: [`Heater.msg`](../../../msg/HeaterCmd.msg) "science/heater_state_data" \
Publisher: science \
Subscriber: gui

#### Spectral Data
Message: [`Spectral.msg`](../../../msg/Spectral.msg) "science/spectral" \
Publisher: science \
Subscriber: gui

#### Spectral Triad Data
Message: [`Triad.msg`](../../../msg/Triad.msg) "science/spectral_triad" \
Publisher: science \
Subscriber: gui

#### Thermistor 0 Data
Message: [Temperature.msg](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Temperature.html) "science/thermistor_0" \
Publisher: science \
Subscriber: gui

#### Thermistor 1 Data
Message: [Temperature.msg](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Temperature.html) "science/thermistor_1" \
Publisher: science \
Subscriber: gui

#### Thermistor 2 Data
Message: [Temperature.msg](https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Temperature.html) "science/thermistor_2" \
Publisher: science \
Subscriber: gui

---

## UART Messages

#### Heater Auto Shut Off Cmd/Data
- `$AUTO_SHUTOFF,<enable>,<extra padding>`
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
- `$SPECTRAL, site, msb_ch0, lsb_ch0, msb_ch1, lsb_ch1, msb_ch2, lsb_ch2, msb_ch3, lsb_ch3, msb_ch4, lsb_ch4, d0_msb_ch5, lsb_ch5, <extra padding>`
- Data is 155 characters long (probably will change tbh)
- 6 channel data from a spectral

#### Servo Cmd
- `$SERVO,<angle_0>,<angle_1>,<angle_2>,<extra padding>`
- Cmd is 155 characters long
- The angles are in degrees

#### Thermistor Data
- `$THERMISTOR,<temp_0>,<temp_1>,<temp_2>,<extra padding>`
- Data is 155 characters long
- Temperature is in Celsius

---

## TODO
- [ ] UPDATE README AND MOVE TO WIKI DOCUMENTATION
- [ ] IMPORTANT - UPDATE SPECTRAL NMEA MESSAGE IN HARDWARE, make sure to also maybe change length of msg
- [ ] simplify the mapper logic (can put into a function instead), don't need both _handler_function_by_tag and _ros_publisherr_by_tag probably
- [ ] Analyze behavior when MOSFET device is out of bounds. See if it should be handled by firmware or here or both.
It is preferred if it is both, but this program does not currently have any checking.
- [ ] Code clean up
- [ ] Move beaglebone stuff into config file
- [ ] Test with ROS
- [ ] Make sure messages and topics are consistent with the gui and teleop programs
- [ ] Perhaps make it so that if there is a hardware mapping problem,
it can be fixed in the config instead of reflashing the firmware (would prob make
life easier if there is a map issue... only problem is that you'll need
to rewrite logic for auto shutoff which can be annoying.)