# GUI LCM Documentation 

### Base Station GUI LCMs

#### All GUIs:

DebugMessage
"/debug_message"
File: DebugMessage.lcm
Subscritions: SATask.vue, AutonTask.vue, RATask.vue
Publishers: N/A
Usage: Sends an error message

Drive Velocity
"/drive_vel_data"
File: DriveVelData.lcm
Subscriptions: DriveVelDataH.vue, DriveVelDataV.vue
Publishers: N/A
Usage: LCM data displayed in table on GUIs

Encoder
"/encoder"
File: Encoder.lcm
Subscriptions: SATask.vue, AutonTask.vue, RATask.vue
Publishers: N/A
Usage: 

NavStatus
"/nav_status"
File: NavStatus.lcm
Subscritions: SATask.vue, AutonTask.vue, RATask.vue
Publishers: N/A
Usage: 

Odometry
"/odometry"
File: OdometryReading.lcm
Subscritions: SATask.vue, AutonTask.vue, RATask.vue
Publishers: N/A
Usage: Displays odometry data on GUIs


#### Science GUI:
MosfetCmd
"/mosfet_cmd"
File: MosfetCmd.lcm
Subscriptions: Chrlophyll.vue
Publishers: Chrlophyll.vue, Amino.vue, OdometryReadingSA.vue
Usage: Enabling and disabling Raman test, UV lights, Thermistor Heaters

SpectralData
"/spectral_data"
File: SpectralData.lcm
Subscriptions: SATask.vue
Publishers: N/A
Usage: Values are passed to SpectralData vue component and read out in table on Science GUI

ThermistorData
"thermistor_data"
ThermistorData.lcm
Subscriptions: Amino.lcm
Publishers: N/A
Usage: Used to read in values for thermistor temperatures

#### Auton GUI:
AutonEnabled
"/auton_enabled"
File: Enable.lcm
Subscriptions: N/A
Publishers: AutonWaypointEditor.vue
Usage: Sends whether autonomous navigation is enabled or not based on the checkbox on the Auton GUI

TeleopEnabled
"/teleop_enabled"
File: Enable.lcm
Subscriptions: N/A
Publishers: AutonWaypointEditor.vue
Usage: Sends whether teleoperation is enabled or not based on the checkbox on the Auton GUI

AutonLed
"/auton_led"
File: AutonLed.lcm
Subscriptions: N/A
Publishers: AutonTask.vue
Usage: Sends what color the navigation state LED should be in order to match the color of the nav state box on the Auton GUI

# ToDo

[] Update README with LCMs (see existing PR #985 in umrover/mrover-workspace)
[] Update Raman Mosfet to be 7 instead of 10
[] Move all ISH box controls to second science GUI
[] Make LCM echo alphabetical
[] Make carousel open loop toggle-able and like gimbal commands
[] Move gimbal commands out of ArmControls and SAArm and into SATask.vue
