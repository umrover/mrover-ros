# script to update and install software dependencies
rosdep update
rosdep install --from-paths ~/catkin_ws/src/ --ignore-src -y --rosdistro=noetic
pip install -r ~/catkin_ws/src/mrover/requirements.txt