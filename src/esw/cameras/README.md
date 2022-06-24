Code to control the USB Cameras
----

### About
For the 2022 Mars Rover Rosie, there are 8 USB cameras. This is the program
responsible for handling service messages received from the base station. \

Up to 4 streams can be requested. The available IPs and video quality of the output streams are determined by the ChangeCameraMission service, which changes the IP of the output streams depending on the mission stated. This information can be located in the config/cameras.yaml file. The ChangeCameras service request has 4 integers representing the device number that the user wants on each of the ports. -1 means no device, and positive integers represent what the jetson recognizes in /dev/video* (e.g. /dev/video0 and /dev/video6). The program does not crash if the video device does not exist.

The program relies on jetson-utils to operate. Its Python functions are called in order to get feed from the camera and render them to a stream. Two pipelines (two streams) are constantly capturing images and rendering them to the output. 

### Services - Server

**Change Camera Mission [Server]** \
Service: [ChangeCameraMission.srv](https://github.com/umrover/mrover-ros/blob/main/srv/ChangeCameraMission.srv "change_camera_mission" \
Server: cameras \
Client: gui \

**Change Cameras [Server]** \
Service: [ChangeCameras.srv](https://github.com/umrover/mrover-ros/blob/main/srv/ChangeCameras.srv) "change_cameras" \
Server: cameras \
Client: gui \

### Usage 

Make sure that you are able to receive streams on the base station by listening to the IPs via GStreamer.
Then you can run the cameras.py program.

### Setup

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

### Issues
Depending on which Jetson you use, you may not be able to stream as many cameras as you'd like at the same time. We use the Jetson Xavier NX, which can only stream up to 4 cameras via USB at a time.

Still need to figure out a way to get camera streams onto the gui.

### Error Handling

### If user requests a camera that does not exist
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

### If the camera disconnects while the device is currently streaming
Currently, if the camera disconnects while the user is streaming it, the program hangs. The current reason is unknown.
To fix this, one should check where it hangs. If it hangs on the Capture function, then one should look into seeing if the video source is available first or not.

The intended behavior is the following: An exception is caught on the Capture function. Then the program resumes as normal.

Actual behavior: The program freezes (presumably on the Capture function).

### ToDo 
- [ ] Standardize naming and order for whether pipeline or device goes first in arguments.
- [ ] Test the following: If an invalid device is being requested, does the response actually say -1 or does it not? Basically, does it only realize the device is invalid too late, as in after it sends the response?
- [ ] Make sure that code makes logical sense... I'm reading through logic and head hurts
- [ ] Test the exact behavior for when "user requests a camera that does not exist" and update README and optimize code.
- [ ] Map physical ports to video so cameras are known by default (low priority)
- [ ] Fix how camera program freezes if USB is disconnected while streaming
- [ ] Low priority - Get camera streams onto the gui

### ToDo - ROS Migration
- [ ] Migrate basics of code that runs on Jetson
- [ ] Migrate basics of code that runs on base station
- [ ] Make sure stuff builds
- [ ] Fix up README.md
- [ ] Figure out how jetson-utils should be included in the system

### ToDo - Pylint
- [ ] Too general exceptions - Figure out what the exact exceptions are
