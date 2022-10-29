#!/usr/bin/env python3

from mrover.msg import CameraCmd
from typing import List, Dict, Tuple

import rospy
from mrover.srv import (
    ChangeCameras,
    ChangeCamerasRequest,
    ChangeCamerasResponse,
)

import jetson.utils


class VideoDevices:

    device: int
    resolution: List[str]
    video_source: jetson.utils.videoSource
    output_by_endpoint: Dict[str, jetson.utils.videoOutput]

    def __init__(self, device: int):
        self.resolution = []
        self.device = device
        self.video_source = None
        self.output_by_endpoint = {}

    def remove_endpoint(self, endpoint: str):
        assert endpoint in self.output_by_endpoint.keys()
        del self.output_by_endpoint[endpoint]
        if len(self.output_by_endpoint) == 0:
            self.video_source = None

    def create_stream(self, endpoint: str, args: List[str]):

        if len(self.output_by_endpoint) == 0 and self.video_source is None:
            # If it does not exist already
            try:
                self.video_source = jetson.utils.videoSource(f"/dev/video{self.device}", argv=args)
                self.output_by_endpoint[endpoint] = jetson.utils.videoOutput(f"rtp://{endpoint}", argv=args)
                self.resolution = args
            except Exception:
                rospy.logerr(f"Failed to create video source for device {self.device}.")
                self.output_by_endpoint = {}
        elif len(self.output_by_endpoint) != 0 and self.video_source is not None:
            # It exists and another stream is using it
            if self.resolution != args:
                # if different args, just recreate video source and every output
                try:
                    self.video_source = jetson.utils.videoSource(f"/dev/video{self.device}", argv=args)
                    for other_endpoint in self.output_by_endpoint.values():
                        self.output_by_endpoint[other_endpoint] = jetson.utils.videoOutput(
                            f"rtp://{other_endpoint}", argv=args
                        )
                    self.output_by_endpoint[other_endpoint] = jetson.utils.videoOutput(f"rtp://{endpoint}", argv=args)
                    self.resolution = args
                except Exception:
                    rospy.logerr(f"Failed to create video source for device {self.device}.")
                    self.output_by_endpoint = {}
            else:
                # If same args, just create a new output
                self.output_by_endpoint[endpoint] = jetson.utils.videoOutput(f"rtp://{endpoint}", argv=args)
        else:
            # This should not happen.
            assert False


class StreamingManager:

    _services: List[List[CameraCmd]]
    _resolution_args: List[List[str]]
    _endpoints: List[str]
    _service_streams_by_endpoints: Dict[str, Tuple[int, int]]
    _video_devices: List[VideoDevices]
    _active_devices: int
    _max_devices: int

    def __init__(self) -> None:

        self._services = [[CameraCmd(-1, 0)] * 4, [CameraCmd(-1, 0)] * 4]
        self._video_devices = []
        self._endpoints = [
            rospy.get_param("cameras/endpoints/primary_0"),
            rospy.get_param("cameras/endpoints/primary_1"),
            rospy.get_param("cameras/endpoints/primary_2"),
            rospy.get_param("cameras/endpoints/primary_3"),
            rospy.get_param("cameras/endpoints/secondary_0"),
            rospy.get_param("cameras/endpoints/secondary_1"),
            rospy.get_param("cameras/endpoints/secondary_2"),
            rospy.get_param("cameras/endpoints/secondary_3"),
        ]
        self._service_streams_by_endpoints = {
            self._endpoints[0]: (0, 0),
            self._endpoints[1]: (0, 1),
            self._endpoints[2]: (0, 2),
            self._endpoints[3]: (0, 3),
            self._endpoints[4]: (1, 0),
            self._endpoints[5]: (1, 1),
            self._endpoints[6]: (1, 2),
            self._endpoints[7]: (1, 3),
        }
        self._resolution_args = [
            rospy.get_param("cameras/arguments/144_res"),
            rospy.get_param("cameras/arguments/360_res"),
            rospy.get_param("cameras/arguments/720_res"),
        ]
        self._active_devices = 0
        self._max_devices = 4  # determined by hardware. this is a constant. do not change.

    def handle_change_cameras(self, req: ChangeCamerasRequest) -> ChangeCamerasResponse:
        camera_commands = req.camera_cmds

        service = self._services[0] if req.primary else self._services[1]
        endpoints = self._endpoints[0:3] if req.primary else self._endpoints[4:7]

        # Only care about turning off streams in first iteration
        for stream, camera_cmd in enumerate(camera_commands):
            endpoint = endpoints[stream]
            requested_device = camera_cmd.device
            requested_resolution = camera_cmd.resolution

            current_device = service[stream].device
            if current_device == requested_device:
                if current_device == -1 or requested_resolution == self._video_devices[requested_device].resolution:
                    continue

            if requested_device == -1:
                previously_was_video_source = self._video_devices[requested_device].video_source is not None
                self._video_devices[current_device].remove_endpoint(endpoint)
                service[stream] = -1
                currently_is_no_video_source = self._video_devices[requested_device].video_source is None
                if previously_was_video_source and currently_is_no_video_source:
                    self._active_devices -= 1

        # after turning off streams, then you can turn on
        for stream, camera_cmd in enumerate(camera_commands):
            endpoint = endpoints[stream]
            requested_device = camera_cmd.device
            requested_resolution = camera_cmd.resolution

            current_device = service[stream].device
            if current_device == requested_device:
                if current_device == -1 or requested_resolution == self._video_devices[requested_device].resolution:
                    continue

            if requested_device != -1:
                if self._active_devices == self._max_devices and current_device == -1:
                    # can not add more than four devices. just continue.
                    continue

                # create a new stream
                previously_no_video_source = self._video_devices[requested_device].video_source is None

                service[stream].device = self._video_devices[requested_device].create_stream(
                    endpoint, self._resolution_args[requested_resolution]
                )
                currently_is_video_source = self._video_devices[requested_device].video_source is not None

                if previously_no_video_source and currently_is_video_source:
                    self._active_devices += 1

        return ChangeCamerasResponse(self._services[0], self._services[1])

    def update_all_streams(self) -> None:
        for index, video_device in enumerate(self._video_devices):
            if video_device.video_source is None:
                continue
            try:
                image = video_device.video_source.Capture(timeout=15000)

                for output in video_device.output_by_endpoint.values():
                    output.Render(image)
            except Exception:
                rospy.logerr(f"Camera {index} capture failed. Stopping stream(s).")
                self._close_down_device(index)

    def _close_down_device(self, device: int):
        previously_was_video_source = self._video_devices[device].video_source is not None
        for endpoint in self._video_devices[device].output_by_endpoint.keys():
            self._video_devices[device].remove_endpoint(endpoint)
            service, stream = self._service_streams_by_endpoints[endpoint]
            self._services[service][stream].device = -1
        currently_is_no_video_source = self._video_devices[device].video_source is None
        if previously_was_video_source and currently_is_no_video_source:
            self._active_devices -= 1


def main():
    rospy.init_node("cameras")
    streaming_manager = StreamingManager()
    rospy.Service("change_cameras", ChangeCameras, streaming_manager.handle_change_cameras)
    while not rospy.is_shutdown():
        streaming_manager.update_all_streams()


if __name__ == "__main__":
    main()
