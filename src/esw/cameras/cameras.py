#!/usr/bin/env python3

"""
This file controls the cameras that the Jetson will stream.
"""
import sys
from enum import Enum
from typing import Any, List

import rospy
from mrover.srv import (ChangeCameraMission, ChangeCameraMissionRequest,
                        ChangeCameraMissionResponse, ChangeCameras,
                        ChangeCamerasRequest, ChangeCamerasResponse)

sys.path.insert(0, "/usr/lib/python3.6/dist-packages")  # 3.6 vs 3.8

import jetson.utils  # noqa


class Mission(Enum):
    "This creates all the mission enums and sets them equal to a number"
    AUTON = 0
    ERD = 1
    ES = 2
    SCIENCE = 3


class Pipeline:
    """Every Pipeline object is control of streaming one
    camera to its assigned ip"""
    _current_ip: str
    _device_number: int
    _video_output: jetson.utils.videoOutput
    _video_source: jetson.utils.videoSource

    def __init__(
            self, arguments: List[str],
            current_mission_ip: str) -> None:
        self._current_ip = current_mission_ip
        self._device_number = -1
        self._video_output = jetson.utils.videoOutput(
            f"rtp://{self._current_ip}",
            argv=arguments)

    def capture_and_render_image(self) -> bool:
        """Captures an image from the video device and streams it.
        Returns bool representing if success or not."""
        try:
            image = self._video_source.Capture()
            self._video_output.Render(image)
            return True
        except Exception:
            # We may expect to catch an exception
            # if the video source does not exist
            return False

    def get_device_number(self) -> int:
        """Returns the camera device which the pipeline is assigned"""
        return self._device_number

    def is_currently_streaming(self) -> bool:
        """Returns whether or not the pipeline is
        assigned an active camera device"""
        return self._device_number != -1

    def update_device_number(
            self, arguments: List[str], index: int,
            video_sources: List[jetson.utils.videoSource]) -> None:
        """Takes in a requested camera device.
        The pipeline will then be assigned this camera device
        unless it does not exist. A -1 means that the pipeline
        is no longer assigned a device."""
        self._device_number = index
        if index != -1:
            video_source = video_sources[index]
            if video_source is not None:
                self._video_output = jetson.utils.videoOutput(
                    f"rtp://{self._current_ip}",
                    argv=arguments)
            else:
                print(f"Unable to play camera {index} on \
                    {self._current_ip}.")
                self._device_number = -1

    def update_video_output(
            self, arguments: List[str],
            current_mission_ip: str) -> None:
        """Updates the video output to ensure that pipeline
        is streaming to the proper IP"""
        self._current_ip = current_mission_ip
        self._video_output = jetson.utils.videoOutput(
                f"rtp://{self._current_ip}",
                argv=arguments)


class PipelineManager:
    _active_cameras: List[int]
    _arguments: List[str]
    _current_mission: str
    _default_mission: str
    _max_vid_dev_id_number: int
    _mission_ips_map: dict[str, str]
    _mission_resolution_map: dict[str, int]
    _pipelines: List[Pipeline]
    _resolution_arguments_map: dict[int, List[str]]
    _video_sources: List[Any]

    def __init__(self) -> None:
        self._resolution_arguments_map = {}
        for input_map in rospy.get_param("cameras/input").items():
            quality = input_map['resolution']
            self._resolution_arguments_map[quality] = input_map['arguments']

        self._default_mission = rospy.get_param("cameras/default_mission")
        self._max_vid_dev_id_number = rospy.get_param(
            "cameras/max_video_device_id_number")

        self._mission_ips_map = {}
        self._mission_resolution_map = {}
        for missions_map in rospy.get_param("cameras/missions").items():
            mission_name = missions_map['name']
            self._mission_ips_map[mission_name] = missions_map['ips']
            self._mission_resolution_map[
                mission_name] = missions_map['resolution']

        self._current_mission = self._default_mission

        quality = self._mission_resolution_map[self._current_mission]
        self._arguments = self._resolution_arguments_map[quality]
        self._video_sources = [None] * self._max_vid_dev_id_number

        number_of_pipelines = rospy.get_param(
            "cameras/number_of_pipelines")
        self._pipelines = [None] * number_of_pipelines
        self._active_cameras = [-1] * number_of_pipelines
        for pipeline_number in range(len(self._pipelines)):
            self._pipelines[pipeline_number] = Pipeline(
                self._arguments,
                self._get_ip(pipeline_number))

    def _start_pipeline(self, index: int, pipeline_number: int) -> None:
        """Takes in index as the camera device and pipeline_number
        as which pipeline number.
        This assigns a camera device to that pipeline."""
        self._pipelines[pipeline_number].update_device_number(
            self._arguments, index, self._video_sources)
        print(f"Playing camera {index} on \
            {self._get_ip(pipeline_number)}.")

    def _close_video_source(self, index: int) -> None:
        """The program will close the connection
        to the video camera at index"""
        self._video_sources[index] = None

    def _create_video_source(self, index: int) -> bool:
        """The program will open a connection to the video camera at index
        unless it does not exist. It returns a bool representing
        whether or not the request was a success."""
        if index == -1:
            return True
        if self._video_sources[index] is not None:
            return True
        try:
            self._video_sources[index] = jetson.utils.videoSource(
                f"/dev/video{index}", argv=self._arguments)
        except Exception:
            # We may expect to catch an exception
            # if the video source does not exist
            return False

    def _get_all_ips(self) -> List[str]:
        return self._mission_ips_map[self._current_mission]

    def _get_ip(self, pipeline_number: int) -> str:
        return self._get_all_ips()[pipeline_number]

    def _pipeline_device_is_unique(
            self, excluded_pipeline: int, device_number: int) -> bool:
        """This function checks whether excluded_pipeline is the
        only pipeline streaming device device_number.
        This function is used to check if any of the other
        pipelines are using the current device."""
        for pipeline_number, pipeline in enumerate(self._pipelines):
            if pipeline_number == excluded_pipeline:
                continue
            if pipeline.get_device_number() == device_number:
                return False
        return True

    def update(self):
        for pipeline_number, pipeline in enumerate(self._pipelines):
            if pipeline.is_currently_streaming():
                success = pipeline.capture_and_render_image()
                if not success:
                    print(f"Camera {pipeline_number} capture \
                        on {self._get_ip(pipeline_number)} \
                        failed. Stopping stream.")
                    failed_device_number = pipeline_number
                    pipeline.update_device_number(
                        self._arguments, -1, self._video_sources)
                    if self._pipeline_device_is_unique(
                            pipeline_number, failed_device_number):
                        self._close_video_source(failed_device_number)

    def handle_change_camera_mission(
            self, req: ChangeCameraMissionRequest) \
            -> ChangeCameraMissionResponse:
        """Handle/callback function to the change
        camera mission service."""
        mission_name = req.mission.lower()
        try:
            current_mission_request = mission_name
            quality = self._mission_resolution_map[current_mission_request]
            self._arguments = self._resolution_arguments_map[quality]
        except KeyError:
            # We may expect to catch an exception
            # if we do not recognize the ChangeCameraMission mission name
            current_mission_request = self._default_mission
            quality = self._mission_resolution_map[current_mission_request]
            self._arguments = self._resolution_arguments_map[quality]
            print(f"Invalid mission name, \
                setting to {self._default_mission}")

        if self._current_mission == current_mission_request:
            return ChangeCameraMissionResponse(self._current_mission)
        self._current_mission = current_mission_request

        for pipeline_number, pipeline in enumerate(self._pipelines):
            # Only skip if 0 or 1 because it's the same either way
            # NOTE: This is an optimization trick made because of how
            # we made the rover. This may change
            # in the future if we decide to make the ips different
            # per mission.
            if pipeline_number == 0 or pipeline_number == 1:
                continue
            pipeline.update_video_output(
                self._arguments, self._get_ip(pipeline_number))

        return ChangeCameraMissionResponse(self._current_mission)

    def handle_change_cameras(
            self, req: ChangeCamerasRequest) -> ChangeCamerasResponse:
        """Handle/callback for changing cameras service"""
        requested_devices = req.cameras

        for ip_number in range(len(self._get_all_ips())):
            # enumerate through current_mission_ips because
            # the length of current_mission_ips is always less
            # than the length of requested_devices.
            pipeline_number = ip_number
            requested_pipeline_device = requested_devices[pipeline_number]
            current_device_number = self._pipelines[
                pipeline_number].get_device_number()

            if current_device_number == requested_pipeline_device:
                continue

            # check if we need to close current video source or not
            if self._pipeline_device_is_unique(
                    pipeline_number, current_device_number):
                self._close_video_source(current_device_number)

            success = self._create_video_source(requested_pipeline_device)
            if not success:
                requested_pipeline_device = -1
            self._start_pipeline(requested_pipeline_device, pipeline_number)

        for pipeline_number, pipeline in enumerate(self._pipelines):
            self._active_cameras[
                pipeline_number] = pipeline.get_device_number()

        return ChangeCamerasResponse(self._active_cameras)


def main():
    """Main function"""
    rospy.init_node("cameras")

    pipeline_manager = PipelineManager()
    rospy.Service('change_cameras', ChangeCameras,
                  pipeline_manager.handle_change_cameras)
    rospy.Service('change_camera_mission', ChangeCameraMission,
                  pipeline_manager.handle_change_camera_mission)
    while not rospy.is_shutdown():
        pipeline_manager.update()


if __name__ == "__main__":
    main()
