#!/usr/bin/env python3
import rospy
import threading

from mrover.msg import EnableAuton
from mrover.srv import PublishEnableAuton, PublishEnableAutonRequest


class AutonBridge:
    """
    A class that manages the state of auton enable.
    """

    def __init__(self):
        """ """
        rospy.wait_for_service("enable_auton")

        self.service_client = rospy.ServiceProxy("enable_auton", PublishEnableAuton, persistent=True)

        self.msg = None
        self.msg_lock = threading.Lock()

    def handle_message(self, msg) -> None:
        """ """
        with self.msg_lock:
            if self.msg == msg:
                return

            self.msg = msg

            try:
                self.service_client(PublishEnableAutonRequest(self.msg))
            except rospy.ServiceException as e:
                rospy.logerr(f"Could not forward enable auton message: {e}")

                self.service_client.close()
                self.service_client = rospy.ServiceProxy("enable_auton", PublishEnableAuton, persistent=True)

                self.msg = None


def main():
    rospy.init_node("auton_enable_forward")

    # Construct the bridge.
    bridge = AutonBridge()

    # Configure subscriber.
    rospy.Subscriber("intermediate_enable_auton", EnableAuton, bridge.handle_message)

    rospy.spin()


if __name__ == "__main__":
    main()
