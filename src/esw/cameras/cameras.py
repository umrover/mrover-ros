#!/usr/bin/env python3

"""Manages the streams for the USB cameras on the rover.

The cameras codebase deals with managing the streams for the USB cameras on
the rover. It manages messages that tell it which devices to stream, at which
quality, and to which endpoints.

"""
import sys
from typing import Dict, List

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
    :var current_endpoint: A string that is endpoint it is assigned to.
    :var _device_number: An int that is the device number it is assigned to as
        its input. -1 means it is not assigned any device.
    :var _video_output: A jetson.utils.videoOutput object that holds its output
        info.
    :var _video_source: A jetson.utils.videoSource object that holds the video
        source info.
    """
    arguments: List[str]
    current_endpoint: str
    _device_number: int
    _video_output: jetson.utils.videoOutput
    _video_source: jetson.utils.videoSource

    def __init__(self) -> None:
        self.arguments = []
        self.current_endpoint = ""
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

        :param dev_index: An integer that is the camera device that it is
            assigned.
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
                    {self.current_endpoint}."
                )
                self.stop_streaming()

    def update_video_output(self) -> None:
        """Updates the video output to ensure that pipeline is streaming to
        the assigned endpoint and has the proper arguments.
        """
        self._video_output = jetson.utils.videoOutput(
            f"rtp://{self.current_endpoint}",
            argv=self.arguments
        )


class PipelineManager:
    """Manages the behavior of all the pipelines.

    :var _active_cameras: A list of integers that is the camera devices
        that are being streaming by each pipeline.
    :var _current_mission: A string that is the current mission. The
        camera qualities and endpoints depend on the mission.
    :var _default_mission: A string that is the default mission to fall
        back to.
    :var _max_vid_dev_id_number: An integer that is the maximum possible
        video devices connected to the Jetson. This determines up to which
        number we can look inside /dev/video*.
    :var _mission_streams_map: A dictionary that maps each mission to a list
        of streams.
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
    _mission_streams_map: 'Dict[str, List[Dict[str, str | int]]]'
    _pipelines: List[Pipeline]
    _res_args_map: Dict[int, List[str]]
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
        self._mission_streams_map = {}
        self._update_mission_streams_map()

        self._current_mission = self._default_mission
        self._video_sources = [None] * self._max_vid_dev_id_number
        number_of_pipelines = rospy.get_param(
            "cameras/number_of_pipelines"
        )
        self._pipelines = [None] * number_of_pipelines
        self._active_cameras = [-1] * number_of_pipelines
        for pipe_index in range(len(self._pipelines)):
            self._pipelines[pipe_index] = Pipeline()

        self._update_all_resolution_arguments()
        self._update_all_endpoints()
        self._update_all_video_outputs()

    def _update_mission_streams_map(self) -> None:
        """Fills in mission endpoints and resolutions maps from cameras.yaml.
        """
        for mission in rospy.get_param("cameras/missions"):
            mission_name = mission['name']
            streams = mission['streams']
            self._mission_streams_map[mission_name] = streams

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
        self._update_all_endpoints()
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
        for stream_index in range(len(self._get_all_streams())):
            requested_device = requested_devices[stream_index]
            if self._is_device_streamed_by_pipe(
                requested_device, stream_index
            ):
                continue
            self._close_device_of_pipeline_if_no_others_are_using(stream_index)
            self._create_video_source_for_pipe_if_possible(
                requested_device, stream_index
            )
            self._stream_device_on_pipe(requested_device, stream_index)

        self._update_active_cameras()

        return ChangeCamerasResponse(self._active_cameras)

    def update(self) -> None:
        """Updates the stream for each pipeline. This means that in order to
        stream feed from a camera, this function must be constantly called.
        """
        for pipe_index, pipeline in enumerate(self._pipelines):
            if pipeline.is_currently_streaming():
                success = pipeline.capture_and_render_image()
                if not success:
                    self._clean_up_failed_pipeline(pipe_index)

    def _clean_up_failed_pipeline(self, pipe_index: int) -> None:
        """Cleans up a pipeline after its device has failed by unassigning it
        to a device and safely closing the stream and the video source.

        :param pipe_index: An integer that is the number of the pipeline that
            has failed.
        """
        failed_device = self._pipelines[pipe_index].device_number
        print(
            f"Camera {failed_device} capture \
            on {self._get_endpoint(pipe_index)} \
            failed. Stopping stream."
        )
        self._stop_all_pipelines_using_this_device(failed_device)

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

        :param pipe_index: An integer that is the number of the pipeline whose
            device is being checked.
        """
        if self._pipeline_device_is_unique(pipe_index):
            pipeline_device_number = (
                self._pipelines[pipe_index].device_number
            )
            self._close_video_source(pipeline_device_number)

    def _close_video_source(self, dev_index: int) -> None:
        """Closes the connection to a video camera by deleting the
        jetson.utils.videoSource.

        :param dev_index: An integer that is the number of the video camera
            device that is being closed.
        """
        self._video_sources[dev_index] = None

    def _create_video_source_for_pipe_if_possible(
        self, dev_index: int, pipe_index: int
    ) -> None:
        """Opens a connection to a video camera by creating the
        jetson.utils.videoSource.

        If the video camera device can not be opened, then an exception is
        caught and the program resumes. The only effect is that the
        videoSource is not made.

        :param dev_index: An integer that is the number of the video camera
            device that is being opened.
        :param pipe_index: An integer that is the number that the device is
            intended to be streamed for.
        """
        if dev_index == -1:
            return
        if self._video_sources[dev_index] is not None:
            return
        try:
            self._video_sources[dev_index] = jetson.utils.videoSource(
                f"/dev/video{dev_index}",
                argv=self._get_pipe_arguments(pipe_index)
            )
        except Exception:
            return

    def _get_all_streams(self) -> 'List[Dict[str, str | int]]':
        """Returns a list of all the streams.

        :return: A list of dictionaries, where the list represents all the
            streams.
        """
        return self._mission_streams_map[self._current_mission]

    def _get_pipe_arguments(self, pipe_index: int) -> List[str]:
        """Returns a list of the current arguments for the pipeline.

        :param pipe_index: An integer that is the number of the pipeline whose
            device is being checked.
        :return: A list of strings that represent the arguments used to create
            the jetson.utils objects.
        """
        streams = self._mission_streams_map[self._current_mission]
        stream = streams[pipe_index]
        quality = stream['resolution']
        return self._res_args_map[quality]

    def _get_stream(self, pipe_index: int) -> 'Dict[str, str | int]':
        """Returns the stream of a pipeline.
        :param pipe_index: An integer that is the number of the pipeline.
        :return: A dictionary that that is the stream info of the pipeline.
        """
        return self._get_all_streams()[pipe_index]

    def _get_endpoint(self, pipe_index: int) -> str:
        """Returns the endpoint of a pipeline.
        :param pipe_index: An integer that is the number of the pipeline.
        :return: A string that that is the endpoint of the pipeline.
        """
        return self._get_stream(pipe_index)['endpoint']

    def _get_pipe_arguments(self, pipe_index: int) -> List[str]:
        """Returns the resolution arguments of a pipeline.
        :param pipe_index: An integer that is the number of the pipeline.
        :return: A string that that is the resolution arguments of a pipeline.
        """
        resolution = self._get_stream(pipe_index)['resolution']
        return self._res_args_map[resolution]

    def _is_mission_name_valid(self, mission_name: str) -> bool:
        """Returns True if the mission_name is valid.

        :param mission_name: A string that is the name of the mission.
            Requires mission_name to be lower case.
        :return: A boolean that tells if the mission name is valid.
        """
        assert mission_name.islower(), "mission_name should be lower case"
        return mission_name in self._mission_streams_map.keys()

    def _is_device_streamed_by_pipe(
        self, dev_index: int, pipe_index: int
    ) -> bool:
        """Returns True if the pipeline is streaming the device.

        :param pipe_index: An integer that is the number of the pipeline.
        :param dev_index: An integer that is the number of the camera device.
        :return: A boolean that tells if the pipeline is currently streaming
            the device.
        """
        pipeline_device_number = self._pipelines[pipe_index].device_number
        return pipeline_device_number == dev_index

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
        for pipe_index, pipeline in enumerate(self._pipelines):
            if pipe_index == excluded_pipe_index:
                continue
            if pipeline.device_number == device_number:
                return False
        return True

    def _stream_device_on_pipe(self, dev_index: int, pipe_index: int) -> None:
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
            {self._get_endpoint(pipe_index)}."
        )

    def _stop_all_pipelines_using_this_device(self, dev_index: int) -> None:
        """
        Stops streams of all pipelines that are streaming a particular
        device.

        This function is called when the device has errored.

        :param dev_index: An integer that makes a pipeline stop streaming if it
            currently streaming that device number.
        """
        self._close_video_source(dev_index)
        for pipeline in self._pipelines:
            if pipeline.device_number == dev_index:
                pipeline.stop_streaming()

    def _update_active_cameras(self) -> None:
        """Updates active cameras being kept track of by modifying
        self._active_cameras.
        """
        for index, pipeline in enumerate(self._pipelines):
            self._active_cameras[index] = pipeline.device_number

    def _update_all_endpoints(self) -> None:
        """Updates the endpoints  to what is currently being requested.

        Only skip if 0 or 1 because it's the same either way.
        NOTE: This is an optimization trick made because of how we made the
        camera system on the rover. This may change in the future if we decide
        to make the first two ips different per mission.
        """
        for pipe_index, pipeline in enumerate(self._pipelines):
            if pipe_index == 0 or pipe_index == 1:
                continue
            pipeline.current_endpoint = self._get_endpoint(pipe_index)

    def _update_all_resolution_arguments(self) -> None:
        """Updates the video resolutions to what is currently being requested.
        """
        for pipe_number, pipeline in enumerate(self._pipelines):
            pipeline.arguments = self._get_pipe_arguments(pipe_number)

    def _update_all_video_outputs(self) -> None:
        """Updates the video outputs and endpoints and video resolutions to
        what is currently being requested.

        Only skip if 0 or 1 because it's the same either way.
        NOTE: This is an optimization trick made because of how we made the
        camera system on the rover. This may change in the future if we decide
        to make the first two endpoints different per mission.
        """
        for pipe_index, pipeline in enumerate(self._pipelines):
            if pipe_index == 0 or pipe_index == 1:
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
