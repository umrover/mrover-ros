# Scripts

---

## Networking Setup

To set up networking on the Jetson, run `source networking_setup_jetson` on the Jetson.

To set up networking on the base station, run `source networking_setup_basestation` on the base station.

This will set up the environment variables needed for the ROS nodes to connect.

The IP of the Jetson should be set to 10.0.0.2 by default.

The IP of the base station should be manually set to include both 10.0.0.7/8 and 10.1.0.7/8.
It needs 10.0.0.7/8 to be on the same network as the Jetson
It needs 10.1.0.7/8 to be on the same network as the radios we use (which are 10.1.0.{3/4/5/6}).

To add an IP, one can run `sudo ip address {add/del} {insert_ip} dev {insert_ethernet_interface}`.
One can find the ethernet interface by running `ip addr`.
It is recommended to add 10.0.0.7/8 before adding 10.1.0.7/8.

For example, one might run `sudo ip address add 10.0.0.7/8 dev enp44s0`.

---