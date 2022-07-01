#!/usr/bin/env python3

"""Manages the streams for the USB cameras on the rover.

The cameras codebase deals with managing the streams for the USB cameras on
the rover. It manages messages that tell it which devices to stream, at which
quality, and to which IPs.

"""
import sys
from typing import List

import rospy
from mrover.srv import (ChangeCameraMission, ChangeCameraMissionRequest,
                        ChangeCameraMissionResponse, ChangeCameras,
                        ChangeCamerasRequest, ChangeCamerasResponse)

sys.path.insert(0, "/usr/lib/python3.6/dist-packages")  # 3.6 vs 3.8

import jetson.utils  # noqa


class Pipeline:
    """Controls the streaming of one camera to its assigned IP.

    The PipelineManager will store multiple of these Pipeline objects
    based on the number of maximum pipelines.

    :var arguments: A list of strings that is needed for the jetson.utils
        objects' capture arguments.
    :var current_ip: A string that is IP it is assigned to.
    :var _device_number: An int that is the device number it is assigned to as
        its input. -1 means it is not assigned any device.
    :var _video_output: A jetson.utils.videoOutput object that holds its output
        info.
    :var _video_source: A jetson.utils.videoSource object that holds the video
        source info.
    """
    arguments: List[str]
    current_ip: str
    _device_number: int
    _video_output: jetson.utils.videoOutput
    _video_source: jetson.utils.videoSource

    def __init__(self) -> None:
        self.arguments = []
        self.current_ip = ""
        self.device_number = -1
        self._video_output = None
        self._video_source = None

    def capture_and_render_image(self) -> bool:
        """Captures an image from the video device and streams it. Returns
        boolean that is the success of capture and render.

        An exception will be caught if the program is unable to capture or
        render while streaming. In this case, False will be returned.

        :return: A boolean that is the success of capture and render.
        """
        try:
            image = self._video_source.Capture()
            self._video_output.Render(image)
            return True
        except Exception:
            return False

    def is_currently_streaming(self) -> bool:
        """Returns whether or not the pipeline is assigned a active camera
            device.

        :return: A boolean that returns True if the pipeline is assigned an
            active camera device.
        """
        return self.device_number != -1

    def stop_streaming(self) -> None:
        """Stops streaming a camera device. This means that the pipeline is
            not assigned to a camera device.
        """
        self.device_number = -1

    def update_device_number(
        self, dev_index: int, video_source: jetson.utils.videoSource
    ) -> None:
        """Assigns the pipeline a camera device. This will also recreate the
        video output for in case the camera device resolution changes. If the
        camera device does not exist or if the request is -1, it will not be
        assigned a device.

        :param dev_index: An int that is the camera device that it is assigned.
        :param video_sources: A jetson.utils.videoSource that it will be
        streaming from. This may be None.
        """
        self.device_number = dev_index
        if dev_index != -1:
            self._video_source = video_source
            if self._video_source is not None:
                self.update_video_output()
            else:
                print(
                    f"Unable to play camera {dev_index} on \
                    {self.current_ip}."
                )
                self.stop_streaming()

    def update_video_output(self) -> None:
        """Updates the video output to ensure that pipeline is streaming to
        the assigned IP and has the proper arguments.
        """
        self._video_output = jetson.utils.videoOutput(
            f"rtp://{self.current_ip}",
            argv=self.arguments
        )


class PipelineManager:
    """Manages the behavior of all the pipelines.

    :var _active_cameras: A list of integers that is the camera devices
        that are being streaming by each pipeline.
    :var _current_mission: A string that is the current mission. The
        camera qualities and IPs depend on the mission.
    :var _default_mission: A string that is the default mission to fall
        back to.
    :var _max_vid_dev_id_number: An integer that is the maximum possible
        video devices connected to the Jetson. This determines up to which
        number we can look inside /dev/video*.
    :var _mission_ips_map: A dictionary that maps each mission to an IP.
    :var _mission_res_map: A dictionary that maps each mission to a resolution
        quality.
    :var _pipelines: A list of Pipeline objects that each manage the streaming
        of a device to an IP.
    :var _res_args_map: A dictionary that maps a resolution quality to a list
        of arguments needed for jetson.utils.
    :var _video_sources: A list of jetson.utils.videoSource's.
    """
    _active_cameras: List[int]
    _current_mission: str
    _default_mission: str
    _max_vid_dev_id_number: int
    _mission_ips_map: dict[str, str]
    _mission_res_map: dict[str, int]
    _pipelines: List[Pipeline]
    _res_args_map: dict[int, List[str]]
    _video_sources: List[jetson.utils.videoSource]

    def __init__(self) -> None:
        self._res_args_map = {}
        for input_map in rospy.get_param("cameras/input").items():
            quality = input_map['resolution']
            self._res_args_map[quality] = input_map['arguments']

        self._default_mission = rospy.get_param("cameras/default_mission")
        self._max_vid_dev_id_number = rospy.get_param(
            "cameras/max_video_device_id_number"
        )

        self._mission_ips_map = {}
        self._mission_res_map = {}
        for missions_map in rospy.get_param("cameras/missions").items():
            mission_name = missions_map['name']
            self._mission_ips_map[mission_name] = missions_map['ips']
            self._mission_res_map[mission_name] = missions_map['resolution']

        self._current_mission = self._default_mission
        self._video_sources = [None] * self._max_vid_dev_id_number
        number_of_pipelines = rospy.get_param(
            "cameras/number_of_pipelines"
        )
        self._pipelines = [None] * number_of_pipelines
        self._active_cameras = [-1] * number_of_pipelines
        for pipeline_number in range(len(self._pipelines)):
            self._pipelines[pipeline_number] = Pipeline()

        self._update_all_resolution_arguments()
        self._update_all_ips()
        self._update_all_video_outputs()

    def handle_change_camera_mission(
        self, req: ChangeCameraMissionRequest
    ) -> ChangeCameraMissionResponse:
        """Processes a request to change the current camera mission.
        Returns the active camera mission after processing the request.

        :param req: A string that is the name of the requested mission.
        :return: A string that represents to the name of the active mission.
            Note that if the requested string was invalid, the mission will
            not have changed and the returned string will be the previous
            mission.
        """
        mission_name = req.mission.lower()
        if self._current_mission == mission_name:
            return ChangeCameraMissionResponse(self._current_mission)
        self._update_mission(mission_name)
        self._update_all_resolution_arguments()
        self._update_all_ips()
        self._update_all_video_outputs()

        return ChangeCameraMissionResponse(self._current_mission)

    def handle_change_cameras(
        self, req: ChangeCamerasRequest
    ) -> ChangeCamerasResponse:
        """Processes a request to change the active cameras. Returns a list of
        the active cameras after processing the request.

        :param req: A list of the requested active cameras that should be
            streamed at each of the pipelines. Note that -1 means no cameras
            should be streaming at that pipeline.
        :return: A list of the active cameras that are being streamed at each
            of the pipelines. Note that -1 means no cameras are streaming at
            that pipeline.
        """
        requested_devices = req.cameras
        for ip_number in range(len(self._get_all_ips())):
            # enumerate through current_mission_ips because
            # the length of current_mission_ips is always less
            # than the length of requested_devices.
            index = ip_number
            requested_device = requested_devices[index]
            if self._is_pipeline_streaming_device(requested_device, index):
                continue
            self._close_device_of_pipeline_if_no_others_are_using(index)
            self._create_video_source_if_possible(requested_device)
            self._start_pipeline(requested_device, index)

        self._update_active_cameras()

        return ChangeCamerasResponse(self._active_cameras)

    def update(self) -> None:
        """Updates the stream for each pipeline. This means that in order to
        stream feed from a camera, this function must be constantly called.
        """
        for pipeline_number, pipeline in enumerate(self._pipelines):
            if pipeline.is_currently_streaming():
                success = pipeline.capture_and_render_image()
                if not success:
                    self._clean_up_failed_pipeline(pipeline_number)

    def _clean_up_failed_pipeline(self, pipe_index: int) -> None:
        """Cleans up a pipeline after its device has failed by unassigning it
        to a device and safely closing the stream and the video source.

        :param pipe_index: the number of the pipeline that has failed.
        """
        failed_device = pipe_index.device_number
        print(
            f"Camera {failed_device} capture \
            on {self._get_ip(pipe_index)} \
            failed. Stopping stream."
        )
        # TODO(SASHREEK) - If others pipelines are using, you should
        # actually close them too. it should be
        # stop_all_pipelines_using_this_device(failed_device).
        self._close_device_of_pipeline_if_no_others_are_using(failed_device)
        self._pipelines[pipe_index].stop_streaming()

    def _close_device_of_pipeline_if_no_others_are_using(
        self, pipe_index: int
    ) -> None:
        """Closes the video source of the current device if no other pipelines
        are using the current video source.

        This assumes that the pipeline at self._pipelines[index] is currently
        assigned a device that is desired to be shutdown. This device will be
        checked against every other pipeline.

        This function is called when a pipeline will be assigned to a
        different camera device. This is done so that if no other pipelines
        are using the device, then it is safely cleaned. If others are using
        the device, nothing will happen.

        :param pipe_index: the number of the pipeline whose device is being
        checked.
        """
        if self._pipeline_device_is_unique(pipe_index):
            pipeline_device_number = (
                self._pipelines[pipe_index].device_number
            )
            self._close_video_source(pipeline_device_number)

    def _close_video_source(self, dev_index: int) -> None:
        """Closes the connection to a video camera by deleting the
        jetson.utils.videoSource.

        :param dev_index: the number of the video camera device that is being
        closed.
        """
        self._video_sources[dev_index] = None

    def _create_video_source_if_possible(self, dev_index: int) -> None:
        """Opens a connection to a video camera by creating the
        jetson.utils.videoSource.

        If the video camera device can not be opened, then an exception is
        caught and the program resumes. The only effect is that the
        videoSource is not made.

        :param dev_index: the number of the video camera device that is being
        opened.
        """
        if dev_index == -1:
            return
        if self._video_sources[dev_index] is not None:
            return
        try:
            self._video_sources[dev_index] = jetson.utils.videoSource(
                f"/dev/video{dev_index}", argv=self._get_current_arguments()
            )
        except Exception:
            return

    def _get_all_ips(self) -> List[str]:
        """Returns a list of all the ips.

        :return: A list of strings that represent the IPs for each pipeline.
        """
        return self._mission_ips_map[self._current_mission]

    def _get_current_arguments(self) -> List[str]:
        """Returns a list of the current arguments.

        :return: A list of strings that represent the arguments used to create
            the jetson.utils objects.
        """
        quality = self._mission_res_map[self._current_mission]
        return self._res_args_map[quality]

    def _get_ip(self, pipe_index: int) -> str:
        """Returns the ip of a pipeline.
        :param pipe_index: An integer that is the number of the pipeline.
        :return: A string that that is the ip of the pipeline.
        """
        return self._get_all_ips()[pipe_index]

    def _is_mission_name_valid(self, mission_name: str) -> bool:
        """Returns True if the mission_name is valid.

        :param mission_name: A string that is the name of the mission.
            Requires mission_name to be lower case.
        :return: A boolean that tells if the mission name is valid.
        """
        assert mission_name.islower(), "mission_name should be lower case"
        return mission_name in self._mission_res_map.keys()

    def _is_pipeline_streaming_device(
        self, pipe_index: int, device: int
    ) -> bool:
        """Returns True if the pipeline is streaming the device.

        :param pipe_index: An integer that is the number of the pipeline.
        :param device: The number of the camera device.
        :return: A boolean that tells if the pipeline is currently streaming
            the device.
        """
        pipeline_device_number = self._pipelines[pipe_index].device_number
        return pipeline_device_number == device

    def _pipeline_device_is_unique(self, excluded_pipe_index: int) -> bool:
        """Returns True if no other pipelines are streaming the same device as
        a particular pipeline.

        :param excluded_pipe_index: An integer that is the number of the
            pipeline whose device is the one that is being compared to the
            devices of other pipelines.
        :return: A boolean that tells if the pipeline is currently streaming
            the device.
        """
        device_number = self._pipelines[excluded_pipe_index].device_number
        for pipeline_number, pipeline in enumerate(self._pipelines):
            if pipeline_number == excluded_pipe_index:
                continue
            if pipeline.device_number == device_number:
                return False
        return True

    def _start_pipeline(self, dev_index: int, pipe_index: int) -> None:
        """Assigns a camera device a pipeline.

        :param dev_index: An integer that is the assigned camera device.
        :param pipe_index: An integer that is the number of the pipeline
            that is being assigned a camera device.
        """
        self._pipelines[pipe_index].update_device_number(
            dev_index, self._video_sources[dev_index]
        )
        print(
            f"Playing camera {dev_index} on \
            {self._get_ip(pipe_index)}."
        )

    def _update_active_cameras(self) -> None:
        """Updates active cameras being kept track of by modifying
        self._active_cameras.
        """
        for index, pipeline in enumerate(self._pipelines):
            self._active_cameras[index] = pipeline.device_number

    def _update_all_ips(self) -> None:
        """Updates the IPs to what is currently being requested.

        Only skip if 0 or 1 because it's the same either way.
        NOTE: This is an optimization trick made because of how we made the
        camera system on the rover. This may change in the future if we decide
        to make the first two ips different per mission.
        """
        for pipeline_number, pipeline in enumerate(self._pipelines):
            if pipeline_number == 0 or pipeline_number == 1:
                continue
            pipeline.current_ip = self._get_ip(pipeline_number)

    def _update_all_resolution_arguments(self) -> None:
        """Updates the video resolutions to what is currently being requested.
        """
        current_arguments = self._get_current_arguments()
        for pipeline in self._pipelines:
            pipeline.arguments = current_arguments

    def _update_all_video_outputs(self) -> None:
        """Updates the video outputs and IPs and video resolutions to what is
        currently being requested.

        Only skip if 0 or 1 because it's the same either way.
        NOTE: This is an optimization trick made because of how we made the
        camera system on the rover. This may change in the future if we decide
        to make the first two ips different per mission.
        """
        for pipeline_number, pipeline in enumerate(self._pipelines):
            if pipeline_number == 0 or pipeline_number == 1:
                continue
            pipeline.update_video_output()

    def _update_mission(self, mission_name: str) -> None:
        """Updates the mission name.

        :param mission_name: A string that is the name of the requested
            mission. Requires mission_name to be lower case.
        """
        assert mission_name.islower(), "mission_name should be lower case"
        if not self._is_mission_name_valid(mission_name):
            print("Invalid mission name. Not changing the mission.")
            return
        self._current_mission = mission_name


def main():
    rospy.init_node("cameras")
    pipeline_manager = PipelineManager()
    rospy.Service(
        'change_cameras', ChangeCameras,
        pipeline_manager.handle_change_cameras
    )
    rospy.Service(
        'change_camera_mission', ChangeCameraMission,
        pipeline_manager.handle_change_camera_mission
    )
    while not rospy.is_shutdown():
        pipeline_manager.update()


if __name__ == "__main__":
    main()
