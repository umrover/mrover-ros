(WIP)

### Code Layout

- [cost_map.cpp](./cost_map.cpp) Mainly ROS node setup (topics, parameters, etc.)
- [cost_map.processing.cpp](./cost_map.processing.cpp) Processes ~~pointclouds~~ occupancy grids to continuously update the global terrain occupancy grid

### Explanation

The goal of this code is to take in a local occupancy grid / costmap (using http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html) and output a global occupancy grid (again using http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html).

~~The goal of this code is to take in pointcloud data from the zed and output a cost map / occupancy grid that represents the difficulty of passing terrain.~~ 

When a new local occupancy grid is received, the nodelet transforms it into the global frame using tf_tree information about the rover's frame relative to a chosen origin. The resolution (size of each grid space in physical units), grid width (in physical units), and grid height (in physical units) is used to determine the size of the costmap and to which grid space a given cost point is used to update. For simplicity, the new grid space's cost is equal to the maximum cost among the point cloud points that fall within the grid space. The cost map is then published for use by navigation nodelets.

~~When a new pointcloud is received, the nodelet first calculates the costs associated with each point in the pointcloud. The cost at a particular location is 100 if the curvature estimate is above our threshold, 0 if it is not, or -1 if there is no data for that location. This discreteness is helpful for the navigation functions using this costmap, as it minimizes erratic movement to account for new data.~~

~~After costs are calculated, each point's location is transformed from the local basis (relative to the rover) to the global basis (relative to the starting location).~~

~~Processed cost point data is then used to update the rover's costmap. The resolution (size of each grid space in physical units), grid width (in physical units), and grid height (in physical units) is used to determine the size of the costmap and to which grid space a given cost point is used to update. For simplicity, the new grid space's cost is equal to the maximum cost among the point cloud points that fall within the grid space. The cost map is then published for use by navigation nodelets.~~

