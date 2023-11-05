(WIP)

### Code Layout

- [cost_map.cpp](./cost_map.cpp) Mainly ROS node setup (topics, parameters, etc.)
- [cost_map.processing.cpp](./cost_map.processing.cpp) Processes pointclouds to continuously update the global terrain cost map

### Explanation

The goal of this code is to take in pointcloud data from the zed and output a cost map / occupancy grid that represents the difficulty of passing terrain. The cost at a particular location is found by the following function:

$ c(x)

To prevent erratic planning, 