## Table of Contents

[Project Overview](#project-overview) \
[Top-Level Code](#top-level-code) \
[Configuration](#configuration) \
[Services - Server](#services---server) \
[Usage](#usage) \
[Setup](#setup) \
[Issues](#issues) \
[Error Handling](#error-handling) \
[Software Requirements](#software-requirements) \
[TODO](#todo) \
[TODO - ROS Migration](#todo---ros-migration)

---

## Project Overview

The cameras codebase deals with managing the streams for the USB cameras on
the rover. It manages messages that tell it which devices to stream, at which
quality, and to which IPs.

---

## Top Level Code

#### [`cameras.py`](./cameras.py)

This program runs main.

#### PipelineManager

One PipelineManager object is created in main to store all the configuration data and
manage the states of the pipelines and various other behavior.

#### Pipeline

Controls the streaming of one camera to its assigned IP.
The PipelineManager will store multiple of these Pipeline objects
based on the number of maximum pipelines.

---

## Configuration

#### [`cameras.yaml`](../../../config/cameras.yaml)

This configuration file allows you to configure various things.

#### Camera Input Resolution Qualities - cameras/input/

You may choose to configure what resolution qualities are available.
The resolution represents the resolution number. The arguments represent
the arguments that are used when creating a jetson.utils object.
You must test to figure out which numbers make sense for the desired resolution.

#### Default Mission - cameras/default_mission

You may choose to configure the default mission. This will change the default mission
on startup.

#### Maximum Video Device ID Number - cameras/max_video_device_id_number

You may choose to configure the maximum video device ID number.
This determines up to which number we can look inside /dev/video*.

#### Available Missions - cameras/missions

You may choose to configure the available missions. Each mission has a name,
a list of ips, and a resolution for each ip. You should not have more ips than
the amount listed in number_of_pipelines (this will cause errors).

#### Number of Pipelines - cameras/number_of_pipelines

You may choose to configure the total number of pipelines that can be streaming at once.
The total number of pipelines must be consistent with the number in ChangeCameras.srv.
It makes sense to keep this number equal to the maximum amount of IPs one would stream
for any given mission.

---

## Services - Server

#### Change Camera Mission
Service: [`ChangeCameraMission.srv`](../../../srv/ChangeCameraMission.srv) "change_camera_mission" \
Server: cameras \
Client: gui \

#### Change Cameras
Service: [`ChangeCameras.srv`](../../../srv/ChangeCameras.srv) "change_cameras" \
Server: cameras \
Client: gui \

---

## Usage

Make sure that you are able to receive streams on the base station by listening to the IPs via GStreamer. Then you can run the cameras.py program.

---

## Setup

In a Docker container, do the equivalent of the following:
Clone [jetson-utils](https://github.com/dusty-nv/jetson-utils) on the Jetson in home directory, make build folder, cd into that and run cmake .. then make. Without the jetson program, you can just run ```./aarch64/bin/video-viewer /dev/video0 rtp://10.0.0.1:5000 --headless``` on the jetson.

```
git clone https://github.com/dusty-nv/jetson-utils
cd jetson-utils
mkdir build
cd build
cmake ..
make
```

Depending on your version of Python, you may need to change the CMakeLists.txt inside the jetson-utils/python/ folder and include your version for the PYTHON_BINDING_VERSIONS. In addition to this, you should edit the jetson/cameras/src/\_\_main\_\_.py file and replace "/usr/lib/python3.6/dist-packages" with your version of Python.  

---

## Issues
Depending on which Jetson you use, you may not be able to stream as many cameras as you'd like at the same time. We use the Jetson Xavier NX, which can only stream up to 4 cameras via USB at a time.

Still need to figure out a way to get camera streams onto the gui.

---

## Error Handling

#### If user requests a camera that does not exist
If the user requests to view the stream from a device but it does not exist, then the code does not crash.
Shown through an example, the behavior in the code is as follows: 
In this example, a ChangeCameras service requests to view /dev/video5 on pipeline 0.
The program will enter the service callback function handle_change_cameras and eventually
enter the create_video_source function. 

Here, one of the following may be happening (need to do more research but not high priority):

1. It will throw an exception
after attempting to create a jetson.utils.videoSource that is caught by
an exception handler. Then, it will just assign no camera to it. 

2. It creates the video source with ease. Then it leaves the service callback
function. Once it heads into the capture_and_render_image function, an exception is caught
once the jetson.utils.videoSource object is unable to run the capture function.

#### If the camera disconnects while the device is currently streaming
Currently, if the camera disconnects while the user is streaming it, the program hangs. The current reason is unknown.
To fix this, one should check where it hangs. If it hangs on the Capture function, then one should look into seeing if the video source is available first or not.

The intended behavior is the following: An exception is caught on the Capture function. Then the program resumes as normal.

Actual behavior: The program freezes (presumably on the Capture function).

---

## Software Requirements

- Can stream the feeds of up to four cameras at the same time.
- Can stream up to a total of four IPs.
- Can stream the feed of one device to multiple IPs.
- Does not crash when camera is unplugged while not streaming.
- Does not crash when camera is unplugged while streaming.
- Properly changes cameras upon ChangeCameras service request.
- Properly sends ChangeCameras service response.
- Properly changes camera mission upon ChangeCameraMission service request.
- Properly sends ChangeCameraMission service response.
- Changing mission allows one to modify IPs and resolutions.
- Can configure IPs used per mission.
- Can configure resolutions used per mission.
- Does not switch cameras when mission is switched.

---

## TODO
- [ ] Might need to add more locks for all pipeline variables
- [ ] Test to make sure that locks don't slow down or bug out the program.
- [ ] Test the following: Are we allowed to copy a jetson.utils.videoSource object? I forgot behavior. Once this is found out, add to documentation/README.
- [ ] Test the following: Test what happens when the one input device is being requested to be used for multiple output streams that differ in the resolution that it requests. Does the program fail and error?
- [ ] Test the following: If an invalid device is being requested, does the response actually say -1 or does it not? Basically, does it only realize the device is invalid too late, as in after it sends the response?
- [ ] Make sure that code makes logical sense... I'm reading through logic and head hurts
- [ ] Test the exact behavior for when "user requests a camera that does not exist" and update README and optimize code.
- [ ] Map physical ports to video so cameras are known by default (low priority)
- [ ] Fix how camera program freezes if USB is disconnected while streaming
- [ ] Low priority - Get camera streams onto the gui
- [ ] Too general exceptions - Figure out what the exact exceptions are for Pylint

---

## TODO - ROS Migration
- [ ] Migrate basics of code that runs on Jetson
- [ ] Migrate basics of code that runs on base station
- [ ] Make sure stuff builds
- [ ] Fix up README.md
- [ ] Figure out how jetson-utils should be included in the system
