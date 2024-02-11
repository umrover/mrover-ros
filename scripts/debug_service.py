#!/usr/bin/env python3
"""
Node for testing service requests for services that are still in development
Logs the service request to stdout
"""

from typing import Any
import rospy
<<<<<<< HEAD
from mrover.srv import ChangeCameras, ChangeCamerasResponse

# Change these values for the service name and type definition to test different values
SERVICE_NAME = "change_cameras"
SERVICE_TYPE = ChangeCameras
=======
from mrover.srv import EnableAuton, EnableAutonResponse

# Change these values for the service name and type definition to test different values
SERVICE_NAME = "enable_auton"
SERVICE_TYPE = EnableAuton
>>>>>>> 6e58e74e38a44673384c89d9dee2dc8284f16c12


def print_service_request(service_request: Any):
    rospy.loginfo(service_request)
<<<<<<< HEAD
    return ChangeCamerasResponse(success=True)
=======
    return EnableAutonResponse(success=True)
>>>>>>> 6e58e74e38a44673384c89d9dee2dc8284f16c12


def main():
    rospy.init_node("debug_service")
    rospy.Service(SERVICE_NAME, SERVICE_TYPE, print_service_request)
    rospy.spin()


if __name__ == "__main__":
    main()
