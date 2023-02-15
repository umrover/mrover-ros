#!/usr/bin/env python3

from mrover.msg import CameraCmd
from typing import List, Dict, Tuple
import cv2
from multiprocessing import Process
import time
import rospy
from mrover.srv import (
    ChangeCameras,
    ChangeCamerasRequest,
    ChangeCamerasResponse,
)


class VideoDevice:

    # Hardware capture settings
    device: int
    width: int
    height: int
    framerate: int
    bitrate: int
    isColored: bool

    video_source: cv2.VideoCapture
    process_by_endpoint: Dict[str, Process]
    alive: bool

    def __init__(self, device: int):
        self.resolution = []
        self.device = device
        self.video_source = None
        self.process_by_endpoint = {}
        self.isColored = True
        self.alive = False

    def set_caps(self, args: List) -> None:
        self.bitrate = int(args[0])
        self.width = int(args[1])
        self.height = int(args[2])
        self.framerate = int(args[3])

    def create_capture(self,endpoint) -> None:
        """
        Constructs video capture source-to-sink pipeline string and captures video from v4l2 hardware.
        """
        if not self.video_source == None:  # exit if capture has already been created
            return
        capstr = (
            "v4l2src device=/dev/video"
            + str(self.device)
            + " do-timestamp=true io-mode=2 ! \
        image/jpeg, width="
            + str(self.width)
            + ", height="
            + str(self.height)
            + ", framerate="
            + str(self.framerate)
            + "/1 ! \
        jpegdec ! \
        videorate ! \
        video/x-raw,\
        framerate="
            + str(self.framerate)
            + "/1 ! \
        nvvidconv ! "
        )
        if self.isColored:
            capstr += " video/x-raw, format=BGRx ! "
        capstr += "videoconvert ! "
        if self.isColored:
            capstr += " video/x-raw, format=BGR ! "
        capstr += "appsink"

        video_source = cv2.VideoCapture(capstr, cv2.CAP_GSTREAMER)
        rospy.logerr(capstr)

        """
        Constructs video stream transmit source-to-sink pipeline string and sends video capture to destination.
        Intended to be used in a new process per endpoint.
        Process start called whenever new endpoint requested
        Process termination called whenever endpoint no longer requested
        :param endpoint: the endpoint (e.g. 10.0.0.7:5000)
        """
        try:
            host = endpoint[0 : len(endpoint) - 5]
            port = int(endpoint[len(endpoint) - 4 : len(endpoint)])

            txstr = "appsrc ! "
            if self.isColored:
                txstr += " video/x-raw, format=BGR ! "
            txstr += "videoconvert ! "
            if self.isColored:
                txstr += " video/x-raw, format=BGRx ! "
            txstr += (
                "nvvidconv ! \
            nvv4l2h264enc \
            bitrate="
                + str(self.bitrate)
                + " ! \
            h264parse ! \
            rtph264pay pt=96 config-interval=1 ! \
            udpsink host="
                + str(host)
                + " port="
                + str(port)
            )
            self.alive = True
            rospy.logerr(txstr)
            out_send = cv2.VideoWriter(
                txstr,
                cv2.CAP_GSTREAMER,
                cv2.VideoWriter_fourcc("H", "2", "6", "4"),
                60,
                (self.width, self.height),
                self.isColored,
            )



            rospy.logerr(
                "\nTransmitting /dev/video"
                + str(self.device)
                + " to "
                + host
                + ":"
                + str(port)
                + " with "
                + str(self.bitrate / 1e6)
                + " Mbps target, "
                + str(self.framerate)
                + " fps target, ("
                + str(self.width)
                + ","
                + str(self.height)
                + ") resolution\n"
            )
            rospy.logerr("checking if transmittable")

            if not video_source.isOpened() or not out_send.isOpened():
                rospy.logerr("\nWARNING: unable to open video source for /dev/video" + str(self.device) + "\n")
                self.remove_endpoint(endpoint)
                exit(0)
            rospy.logerr("about to transmit")
            # Transmit loop
            while True:
                # TODO: When camera disconnects, this loop gets stuck. Implement watchdog thread?
                # rospy.logerr("transmitting")
                # rospy.logerr(self.process_by_endpoint)
                self.alive = True
                ret, frame = video_source.read()
                if not ret:
                    rospy.logerr("empty frame")
                    break
                out_send.write(frame)

            video_source.release()
            out_send.release()
            self.remove_endpoint(endpoint)
        except Exception as e:
            rospy.logerr(f"THERE IS AN EXCEPTION {type(e)}: {e}")

    def remove_endpoint(self, endpoint: str) -> None:
        """
        Removes screen endpoint from video source list.
        If endpoint dictionary is empty, it also deletes video source.
        :param endpoint: the endpoint (e.g. 10.0.0.7:5000)
        """
        rospy.logerr("removing endpoint "+endpoint)
        assert endpoint in self.process_by_endpoint.keys()
        self.process_by_endpoint[endpoint].kill()
        #self.process_by_endpoint[endpoint].join()
        del self.process_by_endpoint[endpoint]
        if len(self.process_by_endpoint) == 0:
            self.video_source = None

    def watchdog(self) -> None:
        """
        Continuously polls the send_capture() loop to watch for hang (can occur upon camera electrical disconnect).
        Intended to be used in a new process per VideoDevice.
        Process start called after a send_capture() to a new endpoint begins.
        Process termination called whenever VideoDevice was is_streaming() previously but no longer is_streaming()
        Watchdog ignored when self.video_source is None.
        """
        while self.video_source is not None:
            # block for a moment
            time.sleep(2)
            # check if send_capture loop hangs
            if not self.alive:
                rospy.logerr(
                    "\nWARNING: unable to open video source for /dev/video"
                    + str(self.device)
                    + ". Camera hardware disconnect?\n"
                )
                # action: remove all endpoints
                for endpoint in self.process_by_endpoint.keys():
                    self.remove_endpoint(endpoint)
            # reset watchdog service
            self.alive = False

    def is_streaming(self) -> bool:
        """
        Returns if streaming or not.
        :return: a bool if streaming
        """
        if len(self.process_by_endpoint) != 0 and self.video_source != None:
            return True
        return False


class StreamingManager:
    _services: List[List[CameraCmd]]
    _resolution_args: List[List[str]]
    _endpoints: List[str]
    _service_streams_by_endpoints: Dict[str, Tuple[int, int]]
    _video_devices: List[VideoDevice]
    _active_devices: int
    _max_devices: int
    #_watchdog_list: List

    def __init__(self):
        self._max_devices = 4
        #self._watchdog_list = []
        self._video_devices = []
        # determined by hardware. this is a constant. do not change.

        self._services = [
            [CameraCmd(-1, 0), CameraCmd(-1, 0), CameraCmd(-1, 0), CameraCmd(-1, 0)],
            [CameraCmd(-1, 0), CameraCmd(-1, 0), CameraCmd(-1, 0), CameraCmd(-1, 0)],
        ]
        for i in range(rospy.get_param("cameras/max_video_device_id_number")):
            skip_every_other_device = rospy.get_param("cameras/skip_every_other_device")
            #self._watchdog_list.append(0)
            if skip_every_other_device:
                self._video_devices.append(VideoDevice(i * 2))
            else:
                self._video_devices.append(VideoDevice(i))
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
            list(rospy.get_param("cameras/arguments/worst_res")),
            list(rospy.get_param("cameras/arguments/low_res")),
            list(rospy.get_param("cameras/arguments/medium_res")),
            list(rospy.get_param("cameras/arguments/high_res")),
            list(rospy.get_param("cameras/arguments/best_res")),
        ]
        self._active_devices = 0

    def handle_change_cameras(self, req: ChangeCamerasRequest) -> ChangeCamerasResponse:
        """
        The callback function for changing cameras.
        :param req: Has information on the requested devices and resolutions for the four streams of a device.
        :return: The processed devices and resolutions for the four streams of the primary and secondary laptops.
        """
        rospy.logerr("ENTER HANDLE CHANGE CAMERAS")
        self._turn_off_streams_based_on_camera_cmd(req.camera_cmds, req.primary)
        self._turn_on_streams_based_on_camera_cmd(req.camera_cmds, req.primary)
        response = ChangeCamerasResponse(self._services[0], self._services[1])

        return response

    def _close_down_all_streams_of_device(self, device: int) -> None:
        """
        Closes down a video device. Usually called if a device crashes while streaming.
        Must make sure that the device lock is locked.
        :param device: The integer ID of the device to be closed.
        """
        previously_was_video_source = self._video_devices[device].is_streaming()
        assert (
            previously_was_video_source
        ), "_close_down_all_streams_of_device should only be called if a streamed device failed"
        while len(self._video_devices[device].process_by_endpoint.keys()) != 0:
            endpoint = list(self._video_devices[device].process_by_endpoint.keys())[0]
            self._video_devices[device].remove_endpoint(endpoint)
            service, stream = self._service_streams_by_endpoints[endpoint]
            self._services[service][stream].device = -1
        #self._watchdog_list[device].kill()
        #self._watchdog_list[device] = 0
        assert self._video_devices[device].video_source is None, "The video source should be None by now"
        self._active_devices -= 1

    def _turn_off_streams_based_on_camera_cmd(
        self, camera_commands: List[CameraCmd], is_req_for_primary_stream: bool
    ) -> None:
        """
        Turns off streams based on camera cmd.
        :param camera_commands: List of CameraCmd showing requested cameras and capture settings.
        :param is_req_for_primary_stream: A bool representing if this is a request for primary laptop or not.
        """
        service_index = 0 if is_req_for_primary_stream else 1
        endpoints = self._endpoints[0:4] if is_req_for_primary_stream else self._endpoints[4:8]

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
                    #self._watchdog_list[previous_device].kill()
                    #self._watchdog_list[previous_device] = 0
                    self._active_devices -= 1

    def _turn_on_streams_based_on_camera_cmd(
        self, camera_commands: List[CameraCmd], is_req_for_primary_stream: bool
    ) -> None:
        """
        Turns on streams based on camera cmd.
        :param camera_commands: List of CameraCmd which show request.
        :param is_req_for_primary_stream: A bool representing if this is a request for primary laptop or not.
        """
        service_index = 0 if is_req_for_primary_stream else 1
        endpoints = self._endpoints[0:4] if is_req_for_primary_stream else self._endpoints[4:8]

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
                self._video_devices[requested_device].set_caps(self._resolution_args[requested_resolution])
                #self._video_devices[requested_device].create_capture()
                self._video_devices[requested_device].process_by_endpoint[endpoint] = Process(
                    target=self._video_devices[requested_device].create_capture, args=(endpoint,)
                )
                #self._watchdog_list[requested_device] = Process(target=self._video_devices[requested_device].watchdog)
                self._video_devices[requested_device].process_by_endpoint[endpoint].start()
                #self._watchdog_list[requested_device].start()
                
                currently_is_video_source = self._video_devices[requested_device].is_streaming()
                self._services[service_index][stream].device = requested_device if currently_is_video_source else -1
                rospy.logerr(self._video_devices[requested_device].process_by_endpoint)
                if previously_no_video_source and currently_is_video_source:
                    self._active_devices += 1


def main():
    rospy.init_node("cameras")
    streaming_manager = StreamingManager()
    rospy.Service("change_cameras", ChangeCameras, streaming_manager.handle_change_cameras)
    rospy.spin()
    # streaming_manager._video_devices[0].set_caps(streaming_manager._resolution_args[4])
    # streaming_manager._video_devices[0].create_capture()
    # proc = Process(target=streaming_manager._video_devices[0].send_capture, args=('10.0.0.7:5000',))
    # proc.start()

    # while not rospy.is_shutdown():
    #     time.sleep(1)
        # for i in range(len(streaming_manager._video_devices)):
        #     rospy.loginfo(streaming_manager._video_devices[i].process_by_endpoint)
        # rospy.loginfo('\n\n')


if __name__ == "__main__":
    main()
