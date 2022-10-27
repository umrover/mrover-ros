#!/usr/bin/env python3

import sys

from mrover.msg import CameraCmd
from typing import List

import rospy
from mrover.srv import (
    ChangeCameras,
    ChangeCamerasRequest,
    ChangeCamerasResponse,
)

sys.path.insert(0, "/usr/lib/python3.8/dist-packages")  # 3.6 vs 3.8

import jetson.utils  # noqa


class PipelineManager:
    def __init__(self) -> None:
        self._video_sources = [None] * rospy.get_param("cameras/max_video_device_id_number")
        self._primary_pipelines = [CameraCmd(-1, 0)] * 4
        self._secondary_pipelines = [CameraCmd(-1, 0)] * 4
        self._resolution_args = [
            rospy.get_param("cameras/arguments/144_res"),
            rospy.get_param("cameras/arguments/360_res"),
            rospy.get_param("cameras/arguments/720_res"),
        ]
        self._primary_endpoints = [
            rospy.get_param("cameras/endpoints/primary_0"),
            rospy.get_param("cameras/endpoints/primary_1"),
            rospy.get_param("cameras/endpoints/primary_2"),
            rospy.get_param("cameras/endpoints/primary_3"),
        ]
        self._secondary_endpoints = [
            rospy.get_param("cameras/endpoints/secondary_0"),
            rospy.get_param("cameras/endpoints/secondary_1"),
            rospy.get_param("cameras/endpoints/secondary_2"),
            rospy.get_param("cameras/endpoints/secondary_3"),
        ]
        self._primary_video_outputs = [None] * 4
        self._secondary_video_outputs = [None] * 4

    def handle_change_cameras(self, req: ChangeCamerasRequest) -> ChangeCamerasResponse:
        camera_cmds = req.cameras

        for stream, camera_cmd in enumerate(camera_cmds):
            device = camera_cmd.device
            resolution = camera_cmd.resolution
            if device != -1:
                resolution_args = self._resolution_args[resolution]
                if self._video_sources[device] is not None:
                    # This means that video source already exists
                    pass
                else:
                    # Attempt to create video source if possible
                    camera_cmds[stream].device = self._create_video_source(device, resolution_args)
                self._primary_video_outputs = self._create_video_output(req.primary, stream, resolution_args)
            elif device == -1:
                # If others are using, then do not do anything
                if device in self._primary_pipelines or device in self._secondary_pipelines:
                    pass
                else:
                    self._video_sources = None

        if req.primary:
            self._primary_pipelines = camera_cmds
        else:
            self._secondary_pipelines = camera_cmds

        return ChangeCamerasResponse(self._primary_pipelines, self._secondary_pipelines)

    def update_all_pipe_streams(self) -> None:
        for stream, camera_cmd in enumerate(self._primary_pipelines):
            device = camera_cmd.device
            success = True
            if device != -1:
                try:
                    image = self._video_sources[device].Capture()
                    self._primary_video_outputs[stream].Render(image)
                except Exception:
                    success = False
            if not success:
                rospy.logerror(
                    f"Camera {device} capture \
                    on {self._primary_endpoints[stream]} \
                    failed. Stopping stream."
                )
                self._stop_all_from_using_device(device)
        for stream, camera_cmd in enumerate(self._secondary_pipelines):
            device = camera_cmd.device
            success = True
            if device != -1:
                try:
                    image = self._video_sources[device].Capture()
                    self._primary_video_outputs[stream].Render(image)
                except Exception:
                    success = False
            if not success:
                rospy.logerror(
                    f"Camera {device} capture \
                    on {self._secondary_endpoints[stream]} \
                    failed. Stopping stream."
                )
                self._stop_all_from_using_device(device)

    def _stop_all_from_using_device(self, device: int) -> None:
        self._video_sources[device] = None
        for stream, camera_cmd in self._primary_pipelines:
            if camera_cmd.device == device:
                self._primary_pipelines[stream].device = -1
                self._primary_video_outputs[stream] = None
        for stream, camera_cmd in self._secondary_pipelines:
            if camera_cmd.device == device:
                self._secondary_pipelines[stream].device = -1
                self._secondary_video_outputs[stream] = None

    def _create_video_output(self, is_primary: bool, stream: int, args: List[str]) -> int:
        try:
            assert len(self.video_info.endpoint), "self.video_info.endpoint should not be empty"
            assert len(self.video_info.arguments), "self.video_info.arguments should not be empty"
            if is_primary:
                self._primary_video_outputs[stream] = jetson.utils.videoOutput(
                    f"rtp://{self._primary_endpoints[stream]}", argv=args
                )
            else:
                self._secondary_video_outputs[stream] = jetson.utils.videoOutput(
                    f"rtp://{self._secondary_endpoints[stream]}", argv=args
                )
        except Exception:
            if is_primary:
                rospy.logerror(
                    f"Update video output failed on endpoint \
                    {self._primary_endpoints[stream]}."
                )
            else:
                rospy.logerror(
                    f"Update video output failed on endpoint \
                    {self._secondary_endpoints[stream]}."
                )

    def _create_video_source(self, device: int, args: List[str]) -> int:
        # return device number if successfully created, otherwise return -1
        try:
            self._video_sources[device] = jetson.utils.videoSource(f"/dev/video{device}", argv=args)
        except Exception:
            rospy.logerror(
                f"Failed to create video source for device \
                {device}."
            )
            return


def main():
    rospy.init_node("cameras")
    pipeline_manager = PipelineManager()
    rospy.Service("change_cameras", ChangeCameras, pipeline_manager.handle_change_cameras)
    while not rospy.is_shutdown():
        pipeline_manager.update_all_pipe_streams()


if __name__ == "__main__":
    main()
