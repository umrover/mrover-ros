#!/usr/bin/env python3

import time
import rospy
from mrover.msg import NetworkBandwidth

# Credit: https://stackoverflow.com/questions/15616378/python-network-bandwidth-monitor


def get_bytes(t: str, interface: str) -> int:
    with open("/sys/class/net/" + interface + "/statistics/" + t + "_bytes", "r") as f:
        data = f.read()
        return int(data)


if __name__ == "__main__":

    rospy.init_node("network_monitor")

    iface = rospy.get_param("network_iface")

    pub = rospy.Publisher("network_bandwidth", NetworkBandwidth, queue_size=1)

    while True:
        tx1 = get_bytes("tx", iface)
        rx1 = get_bytes("rx", iface)

        time.sleep(1)

        tx2 = get_bytes("tx", iface)
        rx2 = get_bytes("rx", iface)

        tx_speed = (tx2 - tx1) * 8.0 / 1000000.0  # Mbps
        rx_speed = (rx2 - rx1) * 8.0 / 1000000.0  # Mbps

        pub.publish(NetworkBandwidth(tx_speed, rx_speed))
