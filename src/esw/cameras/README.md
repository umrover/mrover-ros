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

### ToDo 

- [ ] Map physical ports to video so cameras are known by default
- [ ] Camera program freezes if USB is disconnected while streaming
- [ ] Low priority - Get camera streams onto the gui

### ToDo - ROS Migration
- [ ] Make sure there is error checking, make sure that MISSION_MAP has array number and error checking
- [ ] Migrate basics of code that runs on Jetson
- [ ] Migrate basics of code that runs on base station
- [ ] Make sure stuff builds
- [ ] Fix up README.md
- [ ] Figure out how jetson-utils should be included in the system
- [ ] Enforce that we pass linter

### ToDo - Pylint
- [ ] Too general exceptions - Figure out what the exact exceptions are
- [ ] Globals are bad
