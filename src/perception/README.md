# [Perception](https://github.com/umrover/mrover-ros/wiki/Perception)

### Code Layout

- [tag_detector.cpp](./tag_detector.cpp) Mainly ROS node setup (topics, parameters, etc.)
- [tag_detector.processing.cpp](./tag_detector.processing.cpp) Processes inputs (camera feed and pointcloud) to estimate locations of the ArUco fiducials
