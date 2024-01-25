import rospy
import sys

import cv2 as cv

from mrover.srv import CapturePanorama, CapturePanoramaRequest, CapturePanoramaResponse
from sensor_msgs.point_cloud2 import PointCloud2


def capture_panorama(request: CapturePanoramaRequest) -> CapturePanoramaResponse:
    def received_point_cloud(point: PointCloud2):
        # 1. turn this point cloud into a cv mat
        # 2. add to list of cv mats to create a panorama from
        ...

    image_subscriber = rospy.Subscriber("/camera/left/points", received_point_cloud, queue_size=1)

    # assume the motors move, repeat a couple of times

    # use OpenCV to stich together those images we created

    # return response, put image as first response
    return CapturePanoramaResponse(...)


def main() -> int:
    rospy.init_node("panorama")
    panorama_service = rospy.Service(
        "capture_panorama",
        CapturePanorama,
    )
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
