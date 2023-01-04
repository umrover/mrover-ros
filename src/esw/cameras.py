#!/usr/bin/env python3

from mrover.msg import CameraCmd
from typing import List, Dict, Tuple

import threading

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
    # Screens video source is displayed on
    output_by_endpoint: Dict[str, jetson.utils.videoOutput]

    def __init__(self, device: int):
        self.resolution = []
        self.device = device
        self.video_source = None
        self.output_by_endpoint = {}

    def remove_endpoint(self, endpoint: str) -> None:
        """
        Removes screen endpoint from video source list.
        If endpoint dictionary is empty, it also deletes video source.
        :param endpoint: the endpoint (e.g. 10.0.0.7:5000)
        """
        assert endpoint in self.output_by_endpoint.keys()
        del self.output_by_endpoint[endpoint]
        if len(self.output_by_endpoint) == 0:
            self.video_source = None

    def create_stream(self, endpoint: str, args: List[str]) -> None:
        """
        Adds endpoint to video source list and creates video_source if not existing.
        :param endpoint: the endpoint (e.g. 10.0.0.7:5000)
        :param args: a list of strings that are the arguments
        """
        if self.is_streaming():
            # If it does not exist already
            try:
                self.video_source = jetson.utils.videoSource(f"/dev/video{self.device}", argv=args)
                self.output_by_endpoint[endpoint] = jetson.utils.videoOutput(f"rtp://{endpoint}", argv=args)
                self.resolution = args
            except Exception:
                rospy.logerr(f"Failed to create video source for device {self.device}.")
                if len(self.output_by_endpoint[endpoint]) == 1:
                    self.output_by_endpoint = {}
        else:
            # It exists and another stream is using it
            if self.resolution != args:
                # If different args, just recreate video source and every output
                try:
                    self.video_source = jetson.utils.videoSource(f"/dev/video{self.device}", argv=args)
                    for other_endpoint in self.output_by_endpoint.values():
                        self.output_by_endpoint[other_endpoint] = jetson.utils.videoOutput(
                            f"rtp://{other_endpoint}", argv=args
                        )
                    self.output_by_endpoint[endpoint] = jetson.utils.videoOutput(f"rtp://{endpoint}", argv=args)
                    self.resolution = args
                except Exception:
                    rospy.logerr(f"Failed to create video source for device {self.device}.")
                    self.output_by_endpoint = {}
            else:
                # If same args and endpoint is already being streamed to, then do nothing
                if endpoint in self.output_by_endpoint.keys():
                    return

                # If same args, just create a new output
                self.output_by_endpoint[endpoint] = jetson.utils.videoOutput(f"rtp://{endpoint}", argv=args)

    def is_streaming(self) -> bool:
        """
        Returns if streaming or not
        :return: a bool if streaming
        """
        is_streaming = len(self.output_by_endpoint) != 0
        if is_streaming and self.video_source is not None:
            return True
        elif not is_streaming and self.video_source is None:
            return False
        else:
            rospy.logerr("Should not have entered this state.")
            assert False


class StreamingManager:

    _services: List[List[CameraCmd]]
    _device_lock: threading.Lock
    _resolution_args: List[List[str]]
    _endpoints: List[str]
    _service_streams_by_endpoints: Dict[str, Tuple[int, int]]
    _video_devices: List[VideoDevices]
    _active_devices: int
    _max_devices: int

    def __init__(self):

        self._services = [
            [CameraCmd(-1, 0), CameraCmd(-1, 0), CameraCmd(-1, 0), CameraCmd(-1, 0)],
            [CameraCmd(-1, 0), CameraCmd(-1, 0), CameraCmd(-1, 0), CameraCmd(-1, 0)],
        ]
        self._video_devices = []
        for i in range(rospy.get_param("cameras/max_video_device_id_number")):
            self._video_devices.append(VideoDevices(i))
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
        # determined by hardware. this is a constant. do not change.
        self._max_devices = 4
        self._device_lock = threading.Lock()

    def handle_change_cameras(self, req: ChangeCamerasRequest) -> ChangeCamerasResponse:
        """
        The callback function for changing cameras.
        :param req: Has information on the requested devices and resolutions for the four streams of a device.
        :return: The processed devices and resolutions for the four streams of the primary and secondary laptops.
        """
        #   This function may not be as fast as desired. For example, if previously there was a device
        #   streaming camera 3, but then it does not want 3, but then another stream wants three, then you will
        #   end up turning it off and turning it back on for no reason (could have just swapped streams instead).
        #   A better implementation would prioritize requests that share the same camera as previously.
        #   However, we can expect this program to only change one device at a time, so it is actually fine.

        self._device_lock.acquire()
        camera_commands = req.camera_cmds

        # 1. Sort the vector based on resolutions and also error check if out of bounds device/resolution
        # 2. Turn off all streams (we turn off first in order to not accidentally create too many devices)
        # 3. After turning off streams, then you can turn on

        camera_commands = self._sort_camera_commands_by_resolution_and_error_check(camera_commands)
        self._turn_off_streams_based_on_camera_cmd(camera_commands, req.primary)
        self._turn_on_streams_based_on_camera_cmd(camera_commands, req.primary)

        response = ChangeCamerasResponse(self._services[0], self._services[1])

        self._device_lock.release()

        return response

    def update_all_streams(self) -> None:
        """
        Updates the streams of all devices
        """
        self._device_lock.acquire()
        for index, video_device in enumerate(self._video_devices):
            if video_device.is_streaming():
                try:
                    # TODO - See if timeout=100 is fine. We have tested with 15,000.
                    image = video_device.video_source.Capture(timeout=100)
                    for output in video_device.output_by_endpoint.values():
                        output.Render(image)
                except Exception as e:
                    rospy.logerr(f"Error encountered: {e}")
                    # TODO - figure out what the exception is
                    # TODO - See if this works: Instead of closing, just ignore
                    rospy.logerr(f"Camera {index} capture failed. Will still try to attempt to stream.")
                    # rospy.logerr(f"Camera {index} capture failed. Stopping stream(s).")
                    # self._close_down_device(index)
        self._device_lock.release()

    def _close_down_all_streams_of_device(self, device: int) -> None:
        """
        Closes down a video device. Usually called if a device crashes while streaming.
        Must make sure that the device lock is locked.
        :param device: The integer ID of the device to be closed.
        """
        assert self._device_lock.locked(), "self._device_lock must be locked first."
        previously_was_video_source = self._video_devices[device].is_streaming()
        assert (
            previously_was_video_source
        ), "_close_down_all_streams_of_device should only be called if a streamed device failed"
        while len(self._video_devices[device].output_by_endpoint.keys()) != 0:
            endpoint = list(self._video_devices[device].output_by_endpoint.keys())[0]
            self._video_devices[device].remove_endpoint(endpoint)
            service, stream = self._service_streams_by_endpoints[endpoint]
            self._services[service][stream].device = -1
        assert self._video_devices[device].video_source is None, "The video source should be None by now"
        self._active_devices -= 1

    def _sort_camera_commands_by_resolution_and_error_check(self, camera_commands: List[CameraCmd]) -> CameraCmd:
        """
        Sorts a list of camera commands by resolution. Also checks for invalid requested devices and resolutions
        :param camera_commands: The list of camera commands to be sorted
        :return: The sorted list of camera commands
        """
        # If two or more requests ask for same video source in different resolution,
        # all requests' resolution get set to the one with the lowest resolution. nlogn + n, but n ~ 4

        # TODO - this function does not actually behave properly.
        #   e.g. When running this function with the following, you get the wrong output:
        #   Input: my_list = [CameraCmd(0, 0), CameraCmd(0, 1), CameraCmd(0, 2), CameraCmd(0, 3)]
        #   Expected Output: output = [CameraCmd(0, 0), CameraCmd(0, 0), CameraCmd(0, 0), CameraCmd(0, 0)]
        #   Actual Output: output = [CameraCmd(0, 0), CameraCmd(0, 0), CameraCmd(0, 1), CameraCmd(0, 2)]

        assert self._device_lock.locked(), "self._device_lock must be locked first."

        for stream, camera_cmd in enumerate(camera_commands):
            if camera_cmd.device >= len(self._video_devices):
                rospy.logerr(f"Request device {camera_cmd.device} invalid. Treating as no device instead.")
                camera_cmd.device = -1
            if camera_cmd.resolution >= len(self._resolution_args):
                rospy.logerr(
                    f"Request resolution {camera_cmd.resolution} invalid. Treating as lowest resolution instead."
                )
                camera_cmd.resolution = 0

        requests: List[Tuple[int, int, int]] = [
            (camera_cmd.device, camera_cmd.resolution, stream) for stream, camera_cmd in enumerate(camera_commands)
        ]

        requests = sorted(requests, key=lambda x: (x[0], x[1]))
        for i in range(1, len(requests)):
            if requests[i][0] == requests[i - 1][0] and requests[i][0] != -1:
                camera_commands[requests[i][2]].resolution = camera_commands[requests[i - 1][2]].resolution

        return camera_commands

    def _turn_off_streams_based_on_camera_cmd(
        self, camera_commands: List[CameraCmd], is_req_for_primary_stream: bool
    ) -> None:
        """
        Turns off streams based on camera cmd
        :param camera_commands: List of CameraCmd which show request
        :param is_req_for_primary_stream: A bool representing if this is a request for primary laptop or not
        """

        service_index = 0 if is_req_for_primary_stream else 1
        endpoints = self._endpoints[0:4] if is_req_for_primary_stream else self._endpoints[4:8]

        assert self._device_lock.locked(), "self._device_lock must be locked first."

        for stream, camera_cmd in enumerate(camera_commands):
            endpoint = endpoints[stream]
            requested_device = camera_cmd.device
            requested_resolution = camera_cmd.resolution

            previous_device = self._services[service_index][stream].device
            # skip if device is the same, already closed, or a different resolution
            if previous_device == requested_device:
                if previous_device == -1 or (
                    requested_device != -1 and requested_resolution == self._video_devices[requested_device].resolution
                ):
                    continue

            if requested_device == -1:
                # this means that there was previously a device, but now we don't want the device to be streamed
                assert self._video_devices[previous_device].is_streaming()
                self._video_devices[previous_device].remove_endpoint(endpoint)
                self._services[service_index][stream].device = -1
                currently_is_no_video_source = not self._video_devices[previous_device].is_streaming()
                if currently_is_no_video_source:
                    self._active_devices -= 1

    def _turn_on_streams_based_on_camera_cmd(
        self, camera_commands: List[CameraCmd], is_req_for_primary_stream: bool
    ) -> None:
        """
        Turns on streams based on camera cmd
        :param camera_commands: List of CameraCmd which show request
        :param is_req_for_primary_stream: A bool representing if this is a request for primary laptop or not
        """
        service_index = 0 if is_req_for_primary_stream else 1
        endpoints = self._endpoints[0:4] if is_req_for_primary_stream else self._endpoints[4:8]

        assert self._device_lock.locked(), "self._device_lock must be locked first."

        for stream, camera_cmd in enumerate(camera_commands):
            endpoint = endpoints[stream]
            requested_device = camera_cmd.device
            requested_resolution = camera_cmd.resolution

            previous_device = self._services[service_index][stream].device
            # skip if previous and current requests are -1 or same resolution
            if previous_device == requested_device:
                if previous_device == -1 or (
                    requested_device != -1 and requested_resolution == self._video_devices[requested_device].resolution
                ):
                    continue

            if requested_device != -1:
                if self._active_devices == self._max_devices and previous_device == -1:
                    # can not add more than four devices. just continue.
                    continue

                # create a new stream
                previously_no_video_source = (
                    previous_device == -1 and not self._video_devices[requested_device].is_streaming()
                )

                if previous_device != -1:
                    self._video_devices[previous_device].remove_endpoint(endpoint)
                self._video_devices[requested_device].create_stream(
                    endpoint, self._resolution_args[requested_resolution]
                )
                currently_is_video_source = self._video_devices[requested_device].is_streaming()
                self._services[service_index][stream].device = requested_device if currently_is_video_source else -1

                if previously_no_video_source and currently_is_video_source:
                    self._active_devices += 1


def main():
    rospy.init_node("cameras")
    streaming_manager = StreamingManager()
    rospy.Service("change_cameras", ChangeCameras, streaming_manager.handle_change_cameras)
    while not rospy.is_shutdown():
        streaming_manager.update_all_streams()


if __name__ == "__main__":
    main()
