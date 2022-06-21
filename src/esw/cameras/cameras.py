"""
This file controls the cameras that the Jetson will stream.
"""
import sys
from enum import Enum

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
    camera to its assigned port"""
    def __init__(
            self, arguments: list[str],
            current_mission_ips: list[str],
            pipeline_number: int) -> None:
        self.video_source = None
        self.pipeline_number = pipeline_number
        self.video_output = jetson.utils.videoOutput(
            f"rtp://{self.current_ips[self.pipeline_number]}",
            argv=arguments)
        self.device_number = -1

    def capture_and_render_image(self) -> None:
        """Captures an image from the video device and streams it"""
        try:
            image = self.video_source.Capture()
            self.video_output.Render(image)
        except Exception:
            print(f"Camera capture {self.device_number} \
                on {self.current_ips[self.pipeline_number]} failed. \
                Stopping stream.")
            failed_device_number = self.device_number
            self.device_number = -1
            if self.pipeline_device_is_unique(self.pipeline_number,
                                              failed_device_number):
                self.close_video_source(failed_device_number)

    def get_device_number(self) -> int:
        """Returns the camera device which the pipeline is assigned"""
        return self.device_number

    def is_currently_streaming(self) -> bool:
        """Returns whether or not the pipeline is
        assigned an active camera device"""
        return self.device_number != -1

    def update_device_number(
            self, arguments: list[str], index: int, video_sources) -> None:
        """Takes in a requested camera device.
        The pipeline will then be assigned this camera device
        unless it does not exist. A -1 means that the pipeline
        is no longer assigned a device."""
        self.device_number = index
        if index != -1:
            self.video_source = video_sources[index]
            if self.video_source is not None:
                self.video_output = jetson.utils.videoOutput(
                    f"rtp://{self.current_ips[self.pipeline_number]}",
                    argv=arguments)
            else:
                print(f"Unable to play camera {index} on \
                    {self.current_ips[self.pipeline_number]}.")
                self.device_number = -1
        else:
            self.video_source = None

    def update_video_output(
            self, arguments: list[str],
            current_mission_ips: list[str]) -> None:
        """Updates the video output to ensure that pipeline
        is streaming to the proper IP"""
        self.current_ips = current_mission_ips
        self.video_output = jetson.utils.videoOutput(
                f"rtp://{self.current_ips[self.pipeline_number]}",
                argv=arguments)


class PipelineManager:
    def __init__(self) -> None:
        self.arguments_map = rospy.get_param("cameras/arguments")
        self.default_mission = rospy.get_param("cameras/default_mission")
        self.max_video_device_id_number = \
            rospy.get_param("cameras/max_video_device_id_number")
        self.mission_map = rospy.get_param("cameras/missions")

        self.pipelines = \
            [None] * rospy.get_param("cameras/number_of_pipelines")
        self.current_mission = self.default_mission
        self.current_mission_ips = \
            self.mission_map[self.current_mission]["ips"].values()
        self.arguments = self.arguments_map[self.current_mission]
        self.video_sources = [None] * self.max_video_device_id_number
        for pipeline_number in range(len(self.pipelines)):
            self.pipelines[pipeline_number] = \
                Pipeline(pipeline_number)

    def update(self):
        for pipeline in self.pipelines:
            if pipeline.is_currently_streaming():
                pipeline.capture_and_render_image()

    def start_pipeline(self, index: int, port: int) -> None:
        """Takes in index as the camera device and port as which pipeline number.
        This assigns a camera device to that pipeline."""
        try:
            self.pipelines[port].update_device_number(
                self.arguments, index, self.video_sources)
            print(f"Playing camera {index} on \
                {self.current_mission_ips[port]}.")
        except Exception:
            return

    def close_video_source(self, index: int) -> None:
        """The program will close the connection
        to the video camera at index"""
        self.video_sources[index] = None

    def create_video_source(self, index: int) -> None:
        """The program will open a connection to the video camera at index
        unless it does not exist."""
        if index == -1:
            return
        if self.video_sources[index] is not None:
            return
        try:
            self.video_sources[index] = jetson.utils.videoSource(
                f"/dev/video{index}", argv=self.arguments)
        except Exception:
            return

    def handle_change_camera_mission(
            self, req: ChangeCameraMissionRequest) -> \
            ChangeCameraMissionResponse:
        """Handle/callback function to the change
        camera mission service."""
        mission_name = req.mission.lower()
        try:
            current_mission_request = mission_name
            self.current_mission_ips = \
                self.mission_map[current_mission_request]["ips"].values()
            self.arguments = self.arguments_map[current_mission_request]
        except KeyError:
            current_mission_request = self.default_mission
            self.current_mission_ips = \
                self.mission_map[current_mission_request]["ips"].values()
            self.arguments = self.arguments_map[current_mission_request]
            print(f"invalid mission name, \
                setting to {self.default_mission}")

        if self.current_mission == current_mission_request:
            return ChangeCameraMissionResponse(self.current_mission)
        self.current_mission = current_mission_request

        for pipeline_number, pipeline in enumerate(self.pipelines):
            # only skip if 0 or 1 because it's the same either way
            if pipeline_number == 0 or pipeline_number == 1:
                continue
            pipeline.update_video_output(
                self.arguments, self.current_mission_ips)

        return ChangeCameraMissionResponse(self.current_mission)

    def handle_change_cameras(
            self, req: ChangeCamerasRequest) -> ChangeCamerasResponse:
        """Handle/callback for changing cameras service"""
        camera_devices = req.cameras

        for ip_number in range(len(self.current_mission_ips)):
            # enumerate through self.current_mission_ips because
            # the length of self.current_mission_ips is always less
            # than the length of camera_devices.
            pipeline_number = ip_number
            requested_port_device = camera_devices[pipeline_number]
            current_device_number = \
                self.pipelines[pipeline_number].get_device_number()

            if current_device_number == requested_port_device:
                continue

            # check if we need to close current video source or not
            if self.pipeline_device_is_unique(
                    pipeline_number, current_device_number):
                self.close_video_source(current_device_number)

            self.create_video_source(requested_port_device)
            self.start_pipeline(requested_port_device, pipeline_number)

        active_cameras = [-1] * self.number_of_pipelines
        for pipeline_number, pipeline in enumerate(self.pipelines):
            active_cameras[pipeline_number] = pipeline.get_device_number()

        return ChangeCamerasResponse(active_cameras)

    def pipeline_device_is_unique(
            self, excluded_pipeline: int, device_number: int) -> bool:
        """This function checks whether excluded_pipeline is the
        only pipeline streaming device device_number.
        This function is used to check if any of the other
        pipelines are using the current device."""
        for pipeline_number, pipeline in enumerate(self.pipelines):
            if pipeline_number == excluded_pipeline:
                continue
            if pipeline.get_device_number() == device_number:
                return False
        return True


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
