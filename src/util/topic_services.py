#!/usr/bin/env python3
# Node for teleop-related callback functions

from curses.panel import top_panel
from math import copysign
from struct import pack
import rospy as ros
import rosmsg
import rospkg
from mrover.srv._FetchPackages import FetchPackages, FetchPackagesResponse, FetchPackagesRequest
import subprocess

class TopicServices():
    
    def fetch_packages_service(self, req: FetchPackagesRequest) -> FetchPackagesResponse:
        rospack = rospkg.RosPack()
        res = [p for p, _ in rosmsg.iterate_packages(rospack, ".msg")]
        res.sort()
        return FetchPackagesResponse(res)

def main():
    ros.init_node("topic_services")
    topics = TopicServices()
    ros.Service("topic_services/fetch_packages", FetchPackages, topics.fetch_packages_service)
    ros.spin()


if __name__ == "__main__":
    main()
