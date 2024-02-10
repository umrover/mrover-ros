# Running the test bridge
## Running on your laptop (not on the jetson)

### Step 1: FDCANUSB configuration
Clone the `fdcanusb` GitHub repo (do this outside of the mrover directory, perhaps in `~/Desktop`):<br>
`git clone https://github.com/mjbots/fdcanusb.git`<br>
`cd fdcanusb`<br>
Build the `fdcanusb_daemon`:<br>
`cd sw/`<br>
`g++ -Wall -g -O2 fdcanusb_daemon.cc -o fdcanusb_daemon`<br>
<b>This step above needs to be done only once. When running the test bridge again (e.g. after reboot of laptop), you only need to do steps 2 onwards</b>.<br>
### Step 2: CAN interface
Create a new can interface: <br>
`sudo ip link add dev vcan0 type vcan`<br>
Set the interface `vcan0` to '`up`'<br>
`sudo ip link set vcan0 up`
### Step 3: Plug in the moteus
Set up, turn on and plug the moteus in via USB. Wait a few seconds for the fdcanusb to turn on and register, then continue.<br>
### Step 4: Start the fdcanusb daemon
Run in one terminal window (in the directory `fdcanusb/sw`):<br>
`./fdcanusb_daemon -F -v /dev/fdcanusb vcan0`

### Step 5: ROS services
In another terminal window, run:<br>
`roscore`<br>
In another terminal window, run:<br>
`mrover`<br>
`rosparam load config/esw_devboard.yaml`<br>
`rosrun mrover can_driver_node _interface:=vcan0`<br>

### Step 6: Test bridge
Make sure all of the other processes are running. Open another window, and run:<br>
`mrover`<br>
`catkin build` <b><-- Re-run this command after making changes, to rebuild the code</b><br>
`rosrun mrover brushless_test_bridge`<br>
^^ Once you have all of the previous set up, you only need to re-run this command.
