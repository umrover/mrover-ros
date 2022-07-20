Code to control the brushed motors, using Nucleo STM32F303REs
===============================================
### About

src/motors is responsible for connecting the rover network's ROS interface with the RA/SA system's Nucleo controller i2c interface

main.cpp calls init() on the static ROSHandler class \
main.cpp calls init() on the static ControllerMap class \
main.cpp calls init() on the static I2C class \
main.cpp creates two threads to run an outgoing function and an incoming function
The outgoing function calls on the ROSHandler's handle_outgoing() function every millisecond
The incoming function calls on the ROSHandler's handle_incoming() function continuously

The ControllerMap class creates a hash table of virtual Controller objects from the config file located at "config/motors.yaml".These virtual Controllers are used to contact the physical controller on the rover, across both RA/SA configurations.

The virtual Controller class is defined in Controller.h.\
Virtual Controllers store information about various controller-specific parameters (such as encoder cpr)\
The virtual Controller class also has functions representing the possible transactions that can be had with the physical controller. \
The virtual Controller will not attempt to communicate with its physical controller unless "activated" by an appropriate ROS message relayed by ROSHandler.h
(e.g. A virtual RA Controller will never attempt to communicate with its physical RA controller unless an RA-related ROS message is sent. This is to prevent multiple virtual Controller objects from trying to contact the same physical Controller object.)

ROSHandler.h is responsible for handling incoming and outgoing ROS messages. \
Incoming ROS messages will trigger functions which call the functions on the appropriate virtual Controllers. \
Outgoing ROS messages are triggered by a clock, which query the functions on the appropriate virtual Controllers for data.

I2C.h is responsible for translating communications by virtual Controllers into i2c transactions understood by the linux drivers.

The following watchdog is implemented: If the nucleos do not receive any I2C messages for a given amount of time (currently about 443 ms), then they reset.

### Topics - Subscribers

#### Arm Closed Loop \[Subscriber\] "/arm_closed_loop_cmd"
Message: [ArmPosition.msg](https://github.com/umrover/mrover-ros/blob/main/msg/ArmPosition.msg) \
Publisher: arm_kinematics \
Subscriber: motors

#### Arm Open Loop \[Subscriber\] "/move_arm_open_loop_cmd"
Message: [ArmOpenLoopCmd.msg](https://github.com/umrover/mrover-ros/blob/main/msg/ArmOpenLoopCmd.msg) \
Publisher: teleop \
Subscriber: motors

#### Carousel Closed Loop \[Subscriber\] "/move_carousel_closed_loop_cmd"
Message: [CarouselPosition.msg](https://github.com/umrover/mrover-ros/blob/main/msg/CarouselPosition.msg) \
Publisher: gui \
Subscriber: motors

#### Carousel Open Loop \[Subscriber\] "/move_carousel_open_loop_cmd"
Message: [CarouselOpenLoopCmd.msg](https://github.com/umrover/mrover-ros/blob/main/msg/CarouselOpenLoopCmd.msg) \
Publisher: gui \
Subscriber: motors

#### Hand Open Loop \[Subscriber\] "/move_hand_open_loop_cmd"
Message: [HandCmd.msg](https://github.com/umrover/mrover-ros/blob/main/msg/HandCmd.msg) \
Publisher: teleop \
Subscriber: src/motors

#### Mast Gimbal Open Loop \[Subscriber\] "/move_mast_gimbal_cmd"
Message: [MastGimbalCmd.msg](https://github.com/umrover/mrover-ros/blob/main/msg/MastGimbalCmd.msg) \
Publisher: teleop \
Subscriber: src/motors

#### SA Closed Loop \[Subscriber\] "/sa_closed_loop_cmd"
Message: [SAPosition.msg](https://github.com/umrover/mrover-ros/blob/main/msg/SAPosition.msg) \
Publisher: arm_kinematics \
Subscriber: motors

#### SA Open Loop \[Subscriber\] "/move_sa_open_loop_cmd"
Message: [SAOpenLoopCmd.msg](https://github.com/umrover/mrover-ros/blob/main/msg/SAOpenLoopCmd.msg) \
Publisher: teleop \
Subscriber: motors

#### Science Hand Open Loop \[Subscriber\] "/move_science_hand_open_loop_cmd"
Message: [ScienceHandCmd.msg](https://github.com/umrover/mrover-workspace/blob/rnucleo/rover_msgs/ScienceHandCmd.msg) \
Publisher: teleop \
Subscriber: src/motors

#### Scoop Limit Switch Enable Cmd \[Subscriber\] "/enable_scoop_limit_switch_cmd"
Message: [Enable.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Enable.msg) \
Publisher: gui \
Subscriber: src/motors

#### Zero Carousel Cmd \[Subscriber\] "/zero_carousel_cmd"
Message: [Signal.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Signal.msg) \
Publisher: gui \
Subscriber: src/motors

### Topics - Publishers

#### Arm Joint B Calibration Data \[Publisher\] "/arm_b_calib_data"
Message: [Calibrate.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Calibrate.msg) \
Publisher: motors \
Subscriber: gui + arm_kinematics

#### Arm Pos Data \[Publisher\] "/arm_pos_data"
Message: [ArmPosition.msg](https://github.com/umrover/mrover-ros/blob/main/msg/ArmPosition.msg) \
Publisher: motors \
Subscriber: arm_kinematics + gui

#### SA Pos Data \[Publisher\] "/sa_pos_data"
Message: [SAPosData.msg](https://github.com/umrover/mrover-ros/blob/main/msg/SAPosData.msg) \
Publisher: src/motors \
Subscriber: arm_kinematics + kineval_stencil + gui

#### Carousel Pos Data \[Publisher\] "/carousel_pos_data"
Message: [CarouselData.msg](https://github.com/umrover/mrover-ros/blob/main/msg/CarouselData.msg) \
Publisher: src/motors \
Subscriber: gui

#### Carousel Calibration Data \[Publisher\] "/carousel_calib_data"
Message: [Calibrate.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Calibrate.msg) \
Publisher: src/motors \
Subscriber: gui

#### SA Joint B Calibration Data \[Publisher\] "/sa_b_calib_data"
Message: [Calibrate.msg](https://github.com/umrover/mrover-ros/blob/main/msg/Calibrate.msg) \
Publisher: src/motors \
Subscriber: gui + arm_kinematics

### Watchdog Timeout Period

See above, but the watchdog timeout period is currently set to 443 ms.

### Usage

After initializing the ROS object, I2C bus, and virtual Controller objects, the CLI will only show errors, since printing output to the console is time expensive. A blank CLI is a good thing.

To control the RA/SA through open-loop
* Ensure teleop is running on the same platform
* Ensure gui is running on the base station
* Operate the joysticks with the ES/ERD/SA Task page open on the base station GUI

To control the ES/ERD/SA through closed-loop
* Ensure teleop is running on the same platform
* Ensure arm_kinematics is running on the same platform
* Ensure kineval_stencil is running on the base station
* Input commands into the arm control page on the base station GUI

### Off Nominal Behavior Handling

None so far.

### Common Errors

This routine typically only thows one type of error, when it has issues communicating with the motor nucleos. They will have the form "<command> failed on channel"

#### <Command> failed on channel
The Nucleo that is unresponsive is the first digit of the channel. For example, if the error message says "activate failed on 10", check Nucleo 1. Typically this is because the wiring on the i2c bus has failed and needs to be fixed. Even after wiring is fixed or other related issues have been resolved, the Nucleos may stay unresponsive. To reset the Nucleo, press its reset button. If this does not fix src/motors, you may have to restart src/motors.

These communication errors can be caused by a failure anywhere on the i2c bus, so it is not unlikely that issues with only one Nucleo will cause the all the Nucleos to fail to communicate properly. 

src/motors will continue to attempt i2c bus transactions as commands come in from teleop, but currently has no other way to diagnose/remedy an i2c failure.

#### "Assertation failed" while initializing a virtual controller
There is an issue with mrover-workspace/config/src/motors/Controller.cpp. Resolve the configuration file's issues before running the program.

#### Encoder values resetting
If two devices share the same i2c address but their functions are continuously called, then one may see the encoder values changing. This is because when only one device for a specific i2c address can be live at a time and when each device is first made live, it's quadrature encoder value is adjusted to its absolute encoder value.


### ToDo
- [ ] Migrate to ROS properly
- [ ] Test limit switch code for standard robotic arm
- [ ] Investigate encoder issues
- [ ] Create new shield next year and make sure correct pins are traced
- [ ] Add LCM tests to test.cpp
- [ ] Zero-index after URC + CIRC (after Rosie)
- [ ] Verify wrist encoder CPR
- [ ] Perhaps custom nucleos (to replace the current 3 nucleos taking up so much space)

### ToDo - ROS Migration
- [ ] CREATE LAUNCH FILE SOMEHOW
- [ ] FIX UP CONFIG AND CONTROLLERMAP.H ASAP
- [ ] Use clang format (alt-shift-f on VSCode)
- [ ] See how to fix meson.build (distinction between real main vs test)
- [ ] Make sure project builds
- [ ] Consider whether services should be implemented over topics (probably not)
- [ ] Fix the Usage section

### ToDo - 2023 Migration
- [ ] Make sure we zero-index on the nucleo end (reflash everything)
- [ ] Remove Science Hand lcm once SA system has been figured out

### Notes

To reference the firmware that is loaded onto the Nucleos, [see this] (https://github.com/umrover/embedded-testbench/tree/motor_nucleo/motor_nucleo).
