#!/usr/bin/env python3

from mrover.msg import CameraCmd
from typing import List, Any

import rospy
from mrover.srv import (
    ChangeCameras,
    ChangeCamerasRequest,
    ChangeCamerasResponse,
)

import jetson.utils


class LaptopService:

    camera_commands: List[CameraCmd]
    endpoints: List[str]
    video_outputs: List[jetson.utils.videoOutput]

    def __init__(self, endpoints: List[str]) -> None:
        self.camera_commands = [CameraCmd(-1, 0)] * 4
        self.endpoints = endpoints
        self.video_outputs = [jetson.utils.videoOutput] * 4

    def create_video_output(self, stream: int, args: List[str]) -> None:
        try:
            self.video_outputs[stream] = jetson.utils.videoOutput(f"rtp://{self.endpoints[stream]}", argv=args)
        except Exception:
            rospy.logerr(f"Update video output failed on endpoint {self.endpoints[stream]}.")


class StreamingManager:

    _services: List[LaptopService]
    _resolution_args: List[List[str]]
    _video_sources: List[jetson.utils.videoSource]

    def __init__(self) -> None:

        self._video_sources = [None] * rospy.get_param("cameras/max_video_device_id_number")
        _primary_endpoints = [
            rospy.get_param("cameras/endpoints/primary_0"),
            rospy.get_param("cameras/endpoints/primary_1"),
            rospy.get_param("cameras/endpoints/primary_2"),
            rospy.get_param("cameras/endpoints/primary_3"),
        ]
        _secondary_endpoints = [
            rospy.get_param("cameras/endpoints/secondary_0"),
            rospy.get_param("cameras/endpoints/secondary_1"),
            rospy.get_param("cameras/endpoints/secondary_2"),
            rospy.get_param("cameras/endpoints/secondary_3"),
        ]
        self._services = [LaptopService(_primary_endpoints), LaptopService(_secondary_endpoints)]
        self._resolution_args = [
            rospy.get_param("cameras/arguments/144_res"),
            rospy.get_param("cameras/arguments/360_res"),
            rospy.get_param("cameras/arguments/720_res"),
        ]

    def handle_change_cameras(self, req: ChangeCamerasRequest) -> ChangeCamerasResponse:
<<<<<<< HEAD

=======
>>>>>>> 208d2093a359c29f08c8ce775c14cbafb916713f
        camera_commands = req.camera_cmds

        service = self._services[0] if req.primary else self._services[1]

        for stream, camera_cmd in enumerate(camera_commands):
            device = camera_cmd.device
            resolution = camera_cmd.resolution
            if device != -1:
                resolution_args = self._resolution_args[resolution]
                if self._video_sources[device] is not None:
                    # This means that video source already exists
                    pass
                else:
                    # Attempt to create video source if possible
                    camera_commands[stream].device = self._create_video_source(device, resolution_args)
                service.create_video_output(stream, resolution_args)
            elif device == -1:
                # If others are using, then do not do anything
                others_are_using = False
                for service in self._services:
                    if others_are_using:
                        break
                    for current_camera_cmd in service.camera_commands:
                        if current_camera_cmd.device == device:
                            others_are_using = True
                            break
                if not others_are_using:
                    self._video_sources[device] = None

        self._services[int(req.primary)].camera_commands = camera_commands

        return ChangeCamerasResponse(self._services[0].camera_commands, self._services[1].camera_commands)

    def update_all_streams(self) -> None:
        for service in self._services:
            for stream, camera_cmd in enumerate(service.camera_commands):
                device = camera_cmd.device
                if device != -1:
                    success = True
                    try:
                        image = self._video_sources[device].Capture()
                        service.video_outputs[stream].Render(image)
                    except Exception:
                        success = False
                    if not success:
                        rospy.logerr(
                            f"Camera {device} capture on {service.endpoints[stream]} failed. Stopping stream."
                        )
                        self._stop_all_from_using_device(device)

    def _stop_all_from_using_device(self, device: int) -> None:
        self._video_sources[device] = None
        for service in self._services:
            for stream, camera_cmd in enumerate(service.camera_commands):
                if camera_cmd.device == device:
                    service.camera_commands[stream].device = -1
                    service.video_outputs[stream] = None

    def _create_video_source(self, device: int, args: List[str]) -> int:
        # return device number if successfully created, otherwise return -1
        try:
            self._video_sources[device] = jetson.utils.videoSource(f"/dev/video{device}", argv=args)
        except Exception:
            rospy.logerr(f"Failed to create video source for device {device}.")
            return -1
        return device


def main():
    rospy.init_node("cameras")
    streaming_manager = StreamingManager()
    rospy.Service("change_cameras", ChangeCameras, streaming_manager.handle_change_cameras)
    while not rospy.is_shutdown():
        streaming_manager.update_all_streams()


if __name__ == "__main__":
    main()
