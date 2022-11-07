#!/usr/bin/env python3
# Node for teleop-related callback functions

from typing import List
import rospy as ros
import subprocess
from mrover.srv._FetchPackages import FetchPackages, FetchPackagesResponse, FetchPackagesRequest
from mrover.srv._FetchMessageFromPackage import (
    FetchMessageFromPackage,
    FetchMessageFromPackageResponse,
    FetchMessageFromPackageRequest,
)


class TopicServices:
    def get_bash_output(self, command: List[str]) -> List[str]:
        # Return output from bash terminal as array of strings of each line out ouput
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        output = []
        if process.stdout is not None:
            while process.stdout.readable():
                line = process.stdout.readline().decode("utf-8").strip()
                if line:
                    output.append(line)
                else:
                    break
            return output
        else:
            return ["Error"]

    def fetch_packages_service(self, req: FetchPackagesRequest) -> FetchPackagesResponse:
        packages = self.get_bash_output(["rosmsg", "packages"])
        return FetchPackagesResponse(packages)

    def fetch_messages_for_package_service(
        self, req: FetchMessageFromPackageRequest
    ) -> FetchMessageFromPackageResponse:
        messages = self.get_bash_output(["rosmsg", "package", req.package])
        return FetchMessageFromPackageResponse(messages)


def main():
    ros.init_node("topic_services")
    topics = TopicServices()
    ros.Service("topic_services/fetch_packages", FetchPackages, topics.fetch_packages_service)
    ros.Service(
        "topic_services/fetch_messages_from_package", FetchMessageFromPackage, topics.fetch_messages_for_package_service
    )
    ros.spin()


if __name__ == "__main__":
    main()
