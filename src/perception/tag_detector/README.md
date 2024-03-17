# [Perception](https://github.com/umrover/mrover-ros/wiki/Perception)

### Code Layout


## Short Range Detection
- [tag_detector.cpp](./tag_detector.cpp) Mainly ROS node setup (topics, parameters, etc.)
- [tag_detector.processing.cpp](./tag_detector.processing.cpp) Processes inputs (camera feed and pointcloud) to estimate locations of the ArUco fiducials


## Long Range Detection
- [long_range_tag_detector.cpp](./long_range_tag_detector.cpp) Mainly ROS node setup (topics, parameters, etc.)
- [long_range_tag_detector.processing.cpp](./long_range_tag_detector.processing.cpp) Does all the input processing (ArUco recognition) and publishes the information.
