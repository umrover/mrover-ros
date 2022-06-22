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
    camera to its assigned ip"""
    def __init__(
            self, arguments: list[str],
            current_mission_ip: str,
            pipeline_number: int) -> None:
        self.current_ip = current_mission_ip
        self.device_number = -1
        self.pipeline_number = pipeline_number
        self.video_output = jetson.utils.videoOutput(
            f"rtp://{self.current_ip}",
            argv=arguments)
        self.video_source = None

    def capture_and_render_image(self) -> bool:
        """Captures an image from the video device and streams it.
        Returns bool representing if success or not."""
        try:
            image = self.video_source.Capture()
            self.video_output.Render(image)
            return True
        except Exception:
            # We may expect to catch an exception
            # if the video source does not exist
            return False

    def get_device_number(self) -> int:
        """Returns the camera device which the pipeline is assigned"""
        return self.device_number

    def is_currently_streaming(self) -> bool:
        """Returns whether or not the pipeline is
        assigned an active camera device"""
        return self.device_number != -1

    def update_device_number(
            self, arguments: list[str], index: int) -> None:
        """Takes in a requested camera device.
        The pipeline will then be assigned this camera device
        unless it does not exist. A -1 means that the pipeline
        is no longer assigned a device."""
        self.device_number = index
        if index != -1:
            self.video_source = self.video_source[index]
            if self.video_source is not None:
                self.video_output = jetson.utils.videoOutput(
                    f"rtp://{self.current_ip}",
                    argv=arguments)
            else:
                print(f"Unable to play camera {index} on \
                    {self.current_ip}.")
                self.device_number = -1
        else:
            self.video_source = None

    def update_video_output(
            self, arguments: list[str],
            current_mission_ip: str) -> None:
        """Updates the video output to ensure that pipeline
        is streaming to the proper IP"""
        self.current_ip = current_mission_ip
        self.video_output = jetson.utils.videoOutput(
                f"rtp://{self.current_ip}",
                argv=arguments)


class PipelineManager:
    def __init__(self) -> None:
        self.resolution_arguments_map: dict[str, list[str]] = {}
        for input_map in rospy.get_param("cameras/input").items():
            quality = input_map['resolution']
            self.resolution_arguments_map[quality] = \
                input_map['arguments']

        self.default_mission = rospy.get_param("cameras/default_mission")
        self.max_video_device_id_number = \
            rospy.get_param("cameras/max_video_device_id_number")

        self.mission_ips_map = {}
        self.mission_resolution_map = {}
        for missions_map in rospy.get_param("cameras/missions").items():
            mission_name = missions_map['name']
            self.mission_ips_map[mission_name] = missions_map['ips']
            self.mission_resolution_map[mission_name] = \
                missions_map['resolution']

        self.current_mission = self.default_mission

        self.arguments: list[str] = self.resolution_arguments_map[
            self.mission_resolution_map[self.current_mission]['resolution']]
        self.video_sources = [None] * self.max_video_device_id_number

        number_of_pipelines = \
            rospy.get_param("cameras/number_of_pipelines")
        self.pipelines: list[Pipeline] = \
            [None] * number_of_pipelines
        self.active_cameras: list[Pipeline] = \
            [-1] * number_of_pipelines
        for pipeline_number in range(len(self.pipelines)):
            self.pipelines[pipeline_number] = \
                Pipeline(
                    self.arguments,
                    self.get_ip(pipeline_number),
                    pipeline_number)

    def update(self):
        for pipeline_number, pipeline in enumerate(self.pipelines):
            if pipeline.is_currently_streaming():
                success = pipeline.capture_and_render_image()
                if not success:
                    print(f"Camera {pipeline_number} capture \
                        on {self.get_ip(pipeline_number)} \
                        failed. Stopping stream.")
                    failed_device_number = pipeline_number
                    pipeline.update_device_number(self.arguments, -1)
                    if self.pipeline_device_is_unique(
                            pipeline_number, failed_device_number):
                        self.close_video_source(failed_device_number)

    def start_pipeline(self, index: int, pipeline_number: int) -> None:
        """Takes in index as the camera device and pipeline_number
        as which pipeline number.
        This assigns a camera device to that pipeline."""
        self.pipelines[pipeline_number].update_device_number(
            self.arguments, index)
        print(f"Playing camera {index} on \
            {self.get_ip(pipeline_number)}.")

    def close_video_source(self, index: int) -> None:
        """The program will close the connection
        to the video camera at index"""
        self.video_sources[index] = None

    def create_video_source(self, index: int) -> bool:
        """The program will open a connection to the video camera at index
        unless it does not exist. It returns a bool representing
        whether or not the request was a success."""
        if index == -1:
            return True
        if self.video_sources[index] is not None:
            return True
        try:
            self.video_sources[index] = jetson.utils.videoSource(
                f"/dev/video{index}", argv=self.arguments)
        except Exception:
            # We may expect to catch an exception
            # if the video source does not exist
            return False

    def get_all_ips(self) -> list[str]:
        return self.mission_ips_map[self.current_mission]["ips"]

    def get_ip(self, pipeline_number: int) -> str:
        return self.get_ips()[pipeline_number]

    def handle_change_camera_mission(
            self, req: ChangeCameraMissionRequest) -> \
            ChangeCameraMissionResponse:
        """Handle/callback function to the change
        camera mission service."""
        mission_name = req.mission.lower()
        try:
            current_mission_request = mission_name
            self.arguments = self.resolution_arguments_map[
                self.mission_resolution_map[
                    current_mission_request]['resolution']]
        except KeyError:
            # We may expect to catch an exception
            # if we do not recognize the ChangeCameraMission mission name
            current_mission_request = self.default_mission
            self.arguments = self.resolution_arguments_map[
                self.mission_resolution_map[
                    current_mission_request]['resolution']]
            print(f"Invalid mission name, \
                setting to {self.default_mission}")

        if self.current_mission == current_mission_request:
            return ChangeCameraMissionResponse(self.current_mission)
        self.current_mission = current_mission_request

        for pipeline_number, pipeline in enumerate(self.pipelines):
            # Only skip if 0 or 1 because it's the same either way
            # NOTE: This is an optimization trick made because of how
            # we made the rover. This may change
            # in the future if we decide to make the ips different
            # per mission.
            if pipeline_number == 0 or pipeline_number == 1:
                continue
            pipeline.update_video_output(
                self.arguments, self.get_ip(pipeline_number))

        return ChangeCameraMissionResponse(self.current_mission)

    def handle_change_cameras(
            self, req: ChangeCamerasRequest) -> ChangeCamerasResponse:
        """Handle/callback for changing cameras service"""
        requested_devices = req.cameras

        for ip_number in range(len(self.get_all_ips())):
            # enumerate through current_mission_ips because
            # the length of current_mission_ips is always less
            # than the length of requested_devices.
            pipeline_number = ip_number
            requested_pipeline_device = requested_devices[pipeline_number]
            current_device_number = \
                self.pipelines[pipeline_number].get_device_number()

            if current_device_number == requested_pipeline_device:
                continue

            # check if we need to close current video source or not
            if self.pipeline_device_is_unique(
                    pipeline_number, current_device_number):
                self.close_video_source(current_device_number)

            success = self.create_video_source(requested_pipeline_device)
            if not success:
                requested_pipeline_device = -1
            self.start_pipeline(requested_pipeline_device, pipeline_number)

        for pipeline_number, pipeline in enumerate(self.pipelines):
            self.active_cameras[pipeline_number] = pipeline.get_device_number()

        return ChangeCamerasResponse(self.active_cameras)

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
