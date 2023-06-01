#!/usr/bin/env python3
import rospy
import threading

from mrover.msg import EnableAuton
from mrover.srv import PublishEnableAuton, PublishEnableAutonRequest

from typing import Optional


class AutonBridge:
    """
    A class that manages the state of auton enable. This is necessary because Vue does not have a way to act as a
    persistent service client. We need persistence in order to quickly enable or disable autonomy, so this program
    acts as a bridge to send the messages.
    """

    service_client: rospy.ServiceProxy
    """
    A persistent client to give navigation it's enable and course requests.
    """

    msg: Optional[EnableAuton]
    """
    The last received message from the GUI. None if not available.
    """

    msg_lock: threading.Lock
    """
    Mutex to access msg.
    """

    def __init__(self):
        """
        Construct bridge object.
        """
        self._connect_to_server()

        self.msg = None
        self.msg_lock = threading.Lock()

    def _connect_to_server(self):
        """
        Create a service proxy, waiting as long as necessary for it to be advertised by auton.
        """
        rospy.loginfo("Waiting for navigation to launch...")
        rospy.wait_for_service("enable_auton")

        self.service_client = rospy.ServiceProxy("enable_auton", PublishEnableAuton, persistent=True)
        rospy.loginfo("Navigation service found!")

    def handle_message(self, msg: EnableAuton) -> None:
        """
        Receive an EnableAuton message from teleop and forward to navigation if it's updated.
        """
        with self.msg_lock:
            # Guard against outdated messages.
            if self.msg == msg:
                return

            # Attempt to make service request, updating msg state if successful.
            try:
                self.service_client(PublishEnableAutonRequest(msg))
                self.msg = msg

            # Reconnect service client upon failure.
            except Exception as e:
                rospy.logerr(f"Could not forward enable auton message: {e}")

                self.service_client.close()
                self._connect_to_server()


def main():
    rospy.init_node("auton_enable_forward")

    # Construct the bridge.
    bridge = AutonBridge()

    # Configure subscriber.
    rospy.Subscriber("intermediate_enable_auton", EnableAuton, bridge.handle_message)

    rospy.spin()


if __name__ == "__main__":
    main()
