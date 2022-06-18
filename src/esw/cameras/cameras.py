"""
This file controls the cameras that the jetson will stream.
"""
import sys

import rospy
from mrover.srv import (ChangeCameraMission, ChangeCameraMissionRequest,
                        ChangeCameraMissionResponse, ChangeCameras,
                        ChangeCamerasRequest, ChangeCamerasResponse)

from config import (DEFAULT_MISSION, MAX_VIDEO_DEVICE_ID_NUMBER, MISSION_MAP,
                    NUMBER_OF_PIPELINES)

sys.path.insert(0, "/usr/lib/python3.6/dist-packages")  # 3.6 vs 3.8
import jetson.utils  # noqa

PIPELINES = [None] * NUMBER_OF_PIPELINES

CURRENT_MISSION, CURRENT_MISSION_IPS, ARGUMENTS = MISSION_MAP[DEFAULT_MISSION]

VIDEO_SOURCES = [None] * MAX_VIDEO_DEVICE_ID_NUMBER


class Pipeline:
    """Every Pipeline object is control of streaming one
    camera to its assigned port"""
    def __init__(self, pipeline_number: int) -> None:
        self.current_ips = CURRENT_MISSION_IPS
        self.video_source = None
        self.pipeline_number = pipeline_number
        self.video_output = jetson.utils.videoOutput(
            f"rtp://{self.current_ips[self.pipeline_number]}", argv=ARGUMENTS)
        self.device_number = -1

    def update_video_output(self) -> None:
        """Updates the video output to ensure that pipeline
        is streaming to the proper IP"""
        self.current_ips = CURRENT_MISSION_IPS
        self.video_output = jetson.utils.videoOutput(
                f"rtp://{self.current_ips[self.pipeline_number]}",
                argv=ARGUMENTS)

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
            if pipeline_device_is_unique(self.pipeline_number,
                                         failed_device_number):
                close_video_source(failed_device_number)

    def get_device_number(self) -> int:
        """Returns the camera device which the pipeline is assigned"""
        return self.device_number

    def update_device_number(self, index: int) -> None:
        """Takes in a requested camera device.
        The pipeline will then be assigned this camera device
        unless it does not exist. A -1 means that the pipeline
        is no longer assigned a device."""
        self.device_number = index
        if index != -1:
            self.video_source = VIDEO_SOURCES[index]
            if self.video_source is not None:
                self.video_output = jetson.utils.videoOutput(
                    f"rtp://{self.current_ips[self.pipeline_number]}",
                    argv=ARGUMENTS)
            else:
                print(f"Unable to play camera {index} on \
                    {self.current_ips[self.pipeline_number]}.")
                self.device_number = -1
        else:
            self.video_source = None

    def is_currently_streaming(self) -> bool:
        """Returns whether or not the pipeline is
        assigned an active camera device"""
        return self.device_number != -1


def start_pipeline(index: int, port: int) -> None:
    """Takes in index as the camera device and port as which pipeline number.
    This assigns a camera device to that pipeline."""
    try:
        PIPELINES[port].update_device_number(index)
        print(f"Playing camera {index} on {CURRENT_MISSION_IPS[port]}.")
    except Exception:
        return


def close_video_source(index: int) -> None:
    """The program will close the connection to the video camera at index"""
    VIDEO_SOURCES[index] = None


def create_video_source(index: int) -> None:
    """The program will open a connection to the video camera at index
    unless it does not exist."""
    if index == -1:
        return
    if VIDEO_SOURCES[index] is not None:
        return
    try:
        VIDEO_SOURCES[index] = jetson.utils.videoSource(
            f"/dev/video{index}", argv=ARGUMENTS)
    except Exception:
        return


def pipeline_device_is_unique(
        excluded_pipeline: int, device_number: int) -> bool:
    """This function checks whether excluded_pipeline is the
    only pipeline streaming device device_number.
    This function is used to check if any of the other
    pipelines are using the current device."""
    for pipeline_number, pipeline in enumerate(PIPELINES):
        if pipeline_number == excluded_pipeline:
            continue
        if pipeline.get_device_number() == device_number:
            return False
    return True


def handle_change_camera_mission(
        req: ChangeCameraMissionRequest) -> ChangeCameraMissionResponse:
    """Handle/callback function to the change camera mission service."""
    global CURRENT_MISSION, CURRENT_MISSION_IPS, ARGUMENTS
    mission_name = req.mission
    try:
        current_mission_request, CURRENT_MISSION_IPS, \
            ARGUMENTS = MISSION_MAP[mission_name]
    except KeyError:
        current_mission_request, CURRENT_MISSION_IPS, \
            ARGUMENTS = MISSION_MAP[DEFAULT_MISSION]
        print(f"invalid mission name, setting to {DEFAULT_MISSION}")

    if CURRENT_MISSION == current_mission_request:
        return ChangeCameraMissionResponse(CURRENT_MISSION)
    CURRENT_MISSION = current_mission_request

    for pipeline_number, pipeline in enumerate(PIPELINES):
        # only skip if 0 or 1 because it's the same either way
        if pipeline_number == 0 or pipeline_number == 1:
            continue
        pipeline.update_video_output()

    return ChangeCameraMissionResponse(CURRENT_MISSION)


def handle_change_cameras(
        req: ChangeCamerasRequest) -> ChangeCamerasResponse:
    """Handle/callback for changing cameras service"""
    camera_devices = req.cameras

    for ip_number in range(len(CURRENT_MISSION_IPS)):
        # enumerate through CURRENT_MISSION_IPS because
        # the length of CURRENT_MISSION_IPS is always less
        # than the length of camera_devices.
        pipeline_number = ip_number
        requested_port_device = camera_devices[pipeline_number]
        current_device_number = PIPELINES[pipeline_number].get_device_number()

        if current_device_number == requested_port_device:
            continue

        # check if we need to close current video source or not
        if pipeline_device_is_unique(pipeline_number, current_device_number):
            close_video_source(current_device_number)

        create_video_source(requested_port_device)
        start_pipeline(requested_port_device, pipeline_number)

    active_cameras = [-1] * NUMBER_OF_PIPELINES
    for pipeline_number, pipeline in enumerate(PIPELINES):
        active_cameras[pipeline_number] = pipeline.get_device_number()

    return ChangeCamerasResponse(active_cameras)


def main():
    """Main function"""
    rospy.init_node("cameras")

    for pipeline_number, pipeline in enumerate(PIPELINES):
        pipeline = Pipeline(pipeline_number)

    rospy.Service('change_cameras', ChangeCameras,
                  handle_change_cameras)
    rospy.Service('change_camera_mission', ChangeCameraMission,
                  handle_change_camera_mission)

    while not rospy.is_shutdown():
        for pipeline in PIPELINES:
            if pipeline.is_currently_streaming():
                pipeline.capture_and_render_image()


if __name__ == "__main__":
    main()
