#!/usr/bin/env python3
"""
	Node for testing service requests for services that are still in development
	Logs the service request to stdout
"""

from typing import Any
import rospy
from mrover.srv import ChangeDeviceState

# Change these values for the service name and type definition to test different values
SERVICE_NAME = "/change_heater_auto_shutoff_state"
SERVICE_TYPE = ChangeDeviceState


def printServiceRequest(service_request: Any):
    rospy.loginfo(service_request)


def main():
    rospy.init_node("debug_service")
    rospy.Service(SERVICE_NAME, SERVICE_TYPE, printServiceRequest)
    rospy.spin()


if __name__ == "__main__":
    main()
