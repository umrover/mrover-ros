#!/usr/bin/env python3
# Node for teleop-related callback functions

from curses.panel import top_panel
from math import copysign
from struct import pack
import rospy as ros
import rosmsg
from mrover.srv._FetchPackages import FetchPackages, FetchPackagesResponse
import subprocess

class TopicServices():
    
    def fetch_packages_service(self) -> FetchPackagesResponse:
        process = subprocess.Popen(
            ["rosmsg", "packages" ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT  # Combine stderr and stdout
        )
        packages = []
        while process.stdout.readable():
            output = process.stdout.readline().decode("utf-8").strip()
            if output:
                packages.append(output)
            else:
                break
        return FetchPackagesResponse(packages)

def main():
    ros.init_node("topic_services")
    topics = TopicServices()
    ros.Service("topic_services/fetch_packages", FetchPackages, topics.fetch_packages_service)
    ros.spin()


if __name__ == "__main__":
    main()
