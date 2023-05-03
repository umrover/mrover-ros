#!/usr/bin/env python3

import rospy

import subprocess  # Run new processes
import re  # regex
import cv2
from multiprocessing import Process
from typing import Dict, List, Any
from threading import Lock

from mrover.msg import CameraCmd

from mrover.srv import (
    ChangeCameras,
    ChangeCamerasRequest,
    ChangeCamerasResponse,
    ResetCameras,
    ResetCamerasRequest,
    ResetCamerasResponse,
)

PRIMARY_IP: str = rospy.get_param("cameras/ips/primary")
SECONDARY_IP: str = rospy.get_param("cameras/ips/secondary")

CAPTURE_ARGS: List[Dict[str, int]] = rospy.get_param("cameras/arguments")


def generate_dev_list() -> List[int]:
    """
    Generates an integer list of valid devices X found in /dev/video*, not including those that are MetaData devices.
    It will only get devices X that are VideoCapture devices (can be used for streaming).
    :return: An array of numbers in ascending order representing
    valid Video Capture devices X where X is /dev/videoX.
    """

    # Runs bash script line: `find /dev -iname 'video*' -printf "%f\n"`
    dev_cmd_output = subprocess.run(
        ["find", "/dev", "-iname", "video*", "-printf", "%f\n"], capture_output=True, text=True
    )
    dev_num_list = list()

    # Look through /dev for files with "video*"
    for dev in dev_cmd_output.stdout.splitlines():
        dev_num = re.sub(r"video", "", dev.strip())
        # Runs bash script line: 'v4l2-ctl --list-formats --device /dev/$dev'
        cmd_output = subprocess.run(
            ["v4l2-ctl", "--list-formats", "--device", f"/dev/{dev.strip()}"], capture_output=True, text=True
        )
        # Checks if video* file has [0], [1], etc. to see if it is an actual video capture source
        if re.search(r"\[[0-9]\]", cmd_output.stdout):
            dev_num_list.append(int(dev_num))
    # Sort since by default, it is in descending order instead of ascending order
    dev_num_list.sort()
    return dev_num_list


class Stream:
    """
    An object encapsulating a stream from a camera to an IP endpoint. On initialization, a process
    is spun up to handle the actual streaming. When no more references to self exist, the
    finalizer (defined in __del__()) clean up the process and command.
    """

    _process: Process
    """
    The process sending the stream.
    """

    _cmd: CameraCmd
    """
    A reference to the CameraCmd sent by an IP endpoint.
    """

    primary: bool
    """
    Whether the stream is being sent to the primary IP endpoint.
    """

    port: int
    """
    The port that the stream is streaming to.
    """

    def __init__(self, req: ChangeCamerasRequest, cmd: CameraCmd, port: int):
        self._cmd = cmd
        self._cmd.device = req.camera_cmd.device
        self._cmd.resolution = req.camera_cmd.resolution

        self.primary = req.primary
        self.port = port

        args = CAPTURE_ARGS[self._cmd.resolution]

        self._process = Process(
            target=send,
            args=(
                self._cmd.device,
                PRIMARY_IP if self.primary else SECONDARY_IP,
                5000 + self.port,
                args["bps"],
                args["quality"],
                args["fps"],
                True,
            ),
        )

        self._process.start()

    def __del__(self) -> None:
        """
        Finalizer for stream. Kill the associated process and reset the cmd.
        """
        self._process.kill()
        self._process.join()

        self._cmd.device = -1
        self._cmd.resolution = -1


class StreamManager:

    MAX_STREAMS: int = rospy.get_param("cameras/max_streams")
    """
    The maximum number of streams that can be simultaneously sustained.
    """

    MAX_DEVICE_ID: int = rospy.get_param("cameras/max_device_id")
    """
    The maximum ID of a camera device.
    """

    _stream_by_device: List[Any]
    """
    A Stream object for each possible device connected over USB. Object is None if device is not
    being streamed. References to contained objects should be avoided, since Stream objects rely
    on Python's reference counting to call their finalizers.
    """

    _primary_cmds: List[CameraCmd]
    """
    The commands requested by the primary IP endpoint. The list should always remain populated
    with live objects, and references can be passed to Stream objects to modify the data within.
    """

    _secondary_cmds: List[CameraCmd]
    """
    The commands requested by the secondary IP endpoint.
    """

    def __init__(self):
        self._lock = Lock()

        self._stream_by_device = [None for _ in range(self.MAX_DEVICE_ID)]
        self._primary_cmds = [CameraCmd(-1, -1) for _ in range(StreamManager.MAX_STREAMS)]
        self._secondary_cmds = [CameraCmd(-1, -1) for _ in range(StreamManager.MAX_STREAMS)]

    def reset_streams(self, req: ResetCamerasRequest) -> ResetCamerasResponse:
        """
        Destroy all streams for a particular IP endpoint.
        :param req: Request message from the GUI.
        :return: A corresponding response.
        """
        with self._lock:
            for i in range(len(self._stream_by_device)):
                # If a stream exists and is to the intended IP endpoint...
                if self._stream_by_device[i] is not None and self._stream_by_device[i].primary == req.primary:
                    # Reset the stream object, thus calling the finalizer and killing the process.
                    self._stream_by_device[i] = None

            return ResetCamerasResponse(self._primary_cmds, self._secondary_cmds)

    def _get_open_cmd_slot(self, primary: bool) -> CameraCmd:
        cmds = self._primary_cmds if primary else self._secondary_cmds

        for cmd in cmds:
            if cmd.device == -1:
                return cmd

        assert False, "Could not find CameraCmd slot"

    def _get_change_response(self, success: bool) -> ChangeCamerasResponse:
        return ChangeCamerasResponse(success, self._primary_cmds, self._secondary_cmds)

    def handle_req(self, req: ChangeCamerasRequest) -> ChangeCamerasResponse:
        """
        Handle a basic cameras request by starting, editing, or deleting a stream.
        :param req: Request message from the GUI.
        :return: A corresponding response.
        """

        device_arr = generate_dev_list()
        try:
            device_id = device_arr[req.camera_cmd.device]
        except IndexError:
            rospy.logerr(f"Received invalid camera device ID {req.camera_cmd.device}")
            return self._get_change_response(False)

        if not (0 <= device_id < self.MAX_DEVICE_ID):
            rospy.logerr(f"Camera device ID {device_id} is not supported, max is {self.MAX_DEVICE_ID}")
            return self._get_change_response(False)

        # The client's device is passed into the actual device array, so then we get device_id
        # Then we just update the request to use the actually mapped device_id
        req.camera_cmd.device = device_id

        with self._lock:
            # Reset the stream object if it exists. If it was running, this finalizes the stream
            # object, which in turn kills the process and resets the command.
            self._stream_by_device[device_id] = None

            # If a stream is being requested...
            # (resolution == -1 means a request to cancel stream)
            if 0 <= req.camera_cmd.resolution < len(CAPTURE_ARGS):

                # If we cannot handle any more streams, return False.
                available_port_arr = [True, True, True, True]
                for stream in self._stream_by_device:
                    if stream:
                        available_port_arr[stream.port] = False

                available_port = -1
                for i, port_is_available in enumerate(available_port_arr):
                    if port_is_available:
                        available_port = i
                        break

                if available_port == -1:
                    # Technically, we don't need a double check
                    return self._get_change_response(False)

                # Get a reference to an available slot and give to a new stream.
                cmd_obj = self._get_open_cmd_slot(req.primary)
                self._stream_by_device[device_id] = Stream(req, cmd_obj, available_port)

            return self._get_change_response(True)


def send(device=0, host="10.0.0.7", port=5000, bitrate=4000000, quality=0, fps=30, is_colored=False):
    # Construct video capture pipeline string
    cap_str = f"v4l2src device=/dev/video{device} do-timestamp=true io-mode=2 ! "
    cap_str += f"video/x-raw, format=YUY2 ! videorate ! video/x-raw, framerate={fps}/1 ! nvvidconv ! "
    if is_colored:
        cap_str += " video/x-raw, format=BGRx ! "
    cap_str += "videoconvert ! "
    if is_colored:
        cap_str += " video/x-raw, format=BGR ! "
    cap_str += "appsink"

    # openCV video capture from v4l2 device
    cap_send = cv2.VideoCapture(cap_str, cv2.CAP_GSTREAMER)

    # Construct stream transmit pipeline string
    txstr = "appsrc ! "
    if is_colored:
        txstr += " video/x-raw, format=BGR ! "
    txstr += "videoconvert ! "
    if is_colored:
        txstr += " video/x-raw, format=BGRx ! "
    txstr += (
        "nvvidconv ! \
    nvv4l2h264enc \
    bitrate="
        + str(bitrate)
        + " ! \
    h264parse ! \
    rtph264pay pt=96 config-interval=1 ! \
    udpsink host="
        + str(host)
        + " port="
        + str(port)
    )

    width = cap_send.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap_send.get(cv2.CAP_PROP_FRAME_HEIGHT)

    video_writer_fps = 60
    if width == 3264 and height == 2448:
        # These are the settings for the rock camera.
        # Only support one quality since camera does not work with lower fps and looks horrible at other resolutions
        width = 3264
        height = 2448
        fps = 15
        video_writer_fps = 2
    elif width == 640 and height == 480:
        # These are the settings for the microscope camera.
        if quality == 0:
            width = 160
            height = 120
            fps = 15
        elif quality == 1:
            width = 176
            height = 144
            fps = 15
        elif quality == 2:
            width = 320
            height = 244
            fps = 15
        elif quality == 3:
            width = 352
            height = 288
            fps = 15
        else:
            width = 740
            height = 480
            fps = 25
    elif width == 1280 and height == 720:
        # These are the settings for the standard USB cameras
        if quality == 0:
            width = 320
            height = 240
            fps = 15
        elif quality == 1:
            width = 640
            height = 480
            fps = 15
        elif quality == 2:
            width = 960
            height = 720
            fps = 15
        elif quality == 3:
            width = 1280
            height = 720
            fps = 15
        else:
            width = 1280
            height = 720
            fps = 30

    cap_send.set(cv2.CAP_PROP_FPS, fps)

    rospy.logerr(f"width is {width} and height is {height} and fps is {fps}")

    # openCV stream transmit pipeline with RTP sink
    fourcc = cv2.VideoWriter_fourcc("H", "2", "6", "4")
    out_send = cv2.VideoWriter(
        txstr, cv2.CAP_GSTREAMER, fourcc, video_writer_fps, (int(width), int(height)), is_colored
    )

    rospy.loginfo(
        "\nTransmitting /dev/video"
        + str(device)
        + " to "
        + host
        + ":"
        + str(port)
        + " with "
        + str(float(bitrate) / 1e6)
        + " Mbps target, ("
        + str(width)
        + ","
        + str(height)
        + ") resolution\n"
    )

    if not cap_send.isOpened() or not out_send.isOpened():
        rospy.logerr("\nWARNING: unable to open video source for /dev/video" + str(device) + "\n")
        exit(0)

    # Transmit loop
    while True:

        ret, frame = cap_send.read()
        if not ret:
            rospy.logerr("Empty frame")
            break
        out_send.write(frame)

    cap_send.release()
    out_send.release()


def main():
    rospy.init_node("cameras")

    stream_manager = StreamManager()
    rospy.Service("change_cameras", ChangeCameras, stream_manager.handle_req)
    rospy.Service("reset_cameras", ResetCameras, stream_manager.reset_streams)

    rospy.spin()


if __name__ == "__main__":
    main()
