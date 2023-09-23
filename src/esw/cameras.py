#!/usr/bin/env python3

import rospy

import subprocess  # Run new processes
import re  # regex
import cv2
from multiprocessing import Process
from typing import Dict, List, Any
from threading import Lock

from mrover.msg import AvailableCameras, CameraCmd

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


class CameraTypeInfo:
    class QualityOption:
        def __init__(self, width: int, height: int, fps: int, bps: int):
            self.width: int = width
            self.height: int = height
            self.fps: int = fps
            self.bps: int = bps

    def __init__(self, vendor_id: str, vendor: str, use_jpeg: bool, quality_options: List[QualityOption]):
        self.vendor_id: str = vendor_id
        self.vendor: str = vendor
        self.use_jpeg: bool = use_jpeg
        self.quality_options: List[CameraTypeInfo.QualityOption] = quality_options


CAMERA_TYPE_INFO_BY_NAME: Dict[str, CameraTypeInfo] = {}  # initialized in main()


class AvailableCamera:
    def __init__(self, cam_id: int, cam_type: str):
        self.id: int = cam_id
        self.type: str = cam_type


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


def get_camera_type(video_device: str) -> str:
    """
    Get the camera type of video device
    :param video_device: the video device name (e.g. /dev/video0)
    :return: The name of the camera type
    """
    # Execute the v4l2-ctl command to get the serial number of the device
    output = subprocess.check_output(["udevadm", "info", "--query=all", video_device])

    vendor_id = ""
    vendor = ""

    for line in output.decode().splitlines():
        if line.startswith("E: ID_VENDOR_ID="):
            vendor_id = line.split("=")[1].strip()
        if line.startswith("E: ID_VENDOR="):
            vendor = line.split("=")[1].strip()

    for name in CAMERA_TYPE_INFO_BY_NAME:
        if vendor_id == CAMERA_TYPE_INFO_BY_NAME[name].vendor_id and vendor == CAMERA_TYPE_INFO_BY_NAME[name].vendor:
            return name

    return ""


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

    def __init__(self, req: ChangeCamerasRequest, cmd: CameraCmd, port: int, camera_type: str):
        self._cmd = cmd
        self._cmd.device = req.camera_cmd.device
        self._cmd.resolution = req.camera_cmd.resolution

        self.primary = req.primary
        self.port = port

        self._process = Process(
            target=send,
            args=(
                self._cmd.device,
                PRIMARY_IP if self.primary else SECONDARY_IP,
                5000 + self.port,
                req.camera_cmd.resolution,
                camera_type,
            ),
        )

        self._process.start()

    def is_still_running(self) -> bool:
        """
        Returns whether the stream (and process) is still running.
        :return: Whether stream is running.
        """
        return self._process.is_alive()

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

    _available_cams_pub: rospy.Publisher
    """s
    Publishes the available cameras.
    """

    _available_cameras_lock: Lock
    """
    A lock that protects member variable _available_cameras.
    """

    _available_cameras: List[AvailableCamera]
    """
    List of all the available supported camera devices.
    """

    def __init__(self):
        self._lock = Lock()
        self._available_cameras_lock = Lock()

        self._stream_by_device = [None for _ in range(self.MAX_DEVICE_ID)]
        self._primary_cmds = [CameraCmd(-1, -1) for _ in range(StreamManager.MAX_STREAMS)]
        self._secondary_cmds = [CameraCmd(-1, -1) for _ in range(StreamManager.MAX_STREAMS)]
        self._available_cams_pub = rospy.Publisher("available_cameras", AvailableCameras, queue_size=1)
        self._available_cameras = []
        self._update_available_cameras()

    def _update_available_cameras(self) -> None:
        """
        Update the available cameras
        """
        raw_device_arr = generate_dev_list()
        available_cameras = []

        for device_id in raw_device_arr:
            camera_type = get_camera_type(f"/dev/video{device_id}")

            # If the camera_type is unsupported (""), then ignore it.
            if camera_type != "":
                available_cameras.append(AvailableCamera(device_id, camera_type))

        with self._available_cameras_lock:
            self._available_cameras = available_cameras

    def publish_available_cameras(self, event=None) -> None:
        """
        Publish list of available cameras
        """
        self._update_available_cameras()

        with self._available_cameras_lock:
            device_arr = [available_camera.id for available_camera in self._available_cameras]
            camera_types = [available_camera.type for available_camera in self._available_cameras]

        available_cameras = AvailableCameras(num_available=len(device_arr), camera_types=camera_types)
        self._available_cams_pub.publish(available_cameras)

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

        with self._available_cameras_lock:
            device_arr = [available_camera.id for available_camera in self._available_cameras]
            camera_types = [available_camera.type for available_camera in self._available_cameras]

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

        original_req_dev_id = req.camera_cmd.device
        req.camera_cmd.device = device_id

        with self._lock:
            # Reset the stream object if it exists. If it was running, this finalizes the stream
            # object, which in turn kills the process and resets the command.
            self._stream_by_device[device_id] = None

            # If a stream is being requested...
            # (resolution == -1 means a request to cancel stream)
            if req.camera_cmd.resolution >= 0:
                # If we cannot handle any more streams, return False.
                available_port_arr = [True, True, True, True]
                for i, stream in enumerate(self._stream_by_device):
                    if stream is not None:
                        if stream.is_still_running():
                            available_port_arr[stream.port] = False
                        else:
                            self._stream_by_device[i] = None

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

                self._stream_by_device[device_id] = Stream(
                    req, cmd_obj, available_port, camera_types[original_req_dev_id]
                )

            return self._get_change_response(True)


def send(
    device: int = 0,
    host: str = "10.0.0.7",
    port: int = 5000,
    quality: int = 0,
    camera_type: str = "",
):
    # Construct video capture pipeline string using str.join()

    try:
        best_option = len(CAMERA_TYPE_INFO_BY_NAME[camera_type].quality_options) - 1

        if quality > best_option:
            quality = best_option
        elif quality < 0:
            quality = 0

        width = CAMERA_TYPE_INFO_BY_NAME[camera_type].quality_options[quality].width
        height = CAMERA_TYPE_INFO_BY_NAME[camera_type].quality_options[quality].height
        fps = CAMERA_TYPE_INFO_BY_NAME[camera_type].quality_options[quality].fps
        bitrate = CAMERA_TYPE_INFO_BY_NAME[camera_type].quality_options[quality].bps
        use_jpeg = CAMERA_TYPE_INFO_BY_NAME[camera_type].use_jpeg
    except KeyError:
        rospy.logerr(f"Unsupported camera type {camera_type}")
        assert False

    capture_str_parts = []
    capture_str_parts.append(f"v4l2src device=/dev/video{device} do-timestamp=true io-mode=2 ! ")

    if use_jpeg:
        capture_str_parts.append(
            f"image/jpeg, width={width}, height={height}, framerate={fps}/1 ! "
            f"jpegdec ! videorate ! video/x-raw, framerate={fps}/1 ! nvvidconv ! "
            f"video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
        )
    else:
        capture_str_parts.append(
            f"videorate ! video/x-raw, width={width}, height={height}, framerate={fps}/1 ! videoconvert ! appsink"
        )

    # openCV video capture from v4l2 device
    cap_send = cv2.VideoCapture("".join(capture_str_parts), cv2.CAP_GSTREAMER)

    # Construct stream transmit pipeline string
    txstr = (
        "appsrc ! video/x-raw, format=BGR ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! "
        f"nvv4l2h264enc bitrate={bitrate} ! h264parse ! rtph264pay pt=96 config-interval=1 ! "
        f"udpsink host={host} port={port}"
    )

    # We need to set with proper width instead of desired width, otherwise it's very slow when capturing images
    # We can just capture the images at the proper width and convert them later.

    # openCV stream transmit pipeline with RTP sink
    fourcc = cv2.VideoWriter_fourcc("H", "2", "6", "4")
    out_send = cv2.VideoWriter(txstr, cv2.CAP_GSTREAMER, fourcc, 60, (int(width), int(height)), True)

    rospy.loginfo(
        f"\nTransmitting /dev/video{str(device)} to {host}: {str(port)} with {str(float(bitrate) / 1e6)} Mbps target, "
        f"({str(width)}, {str(height)}) resolution\n"
    )

    if not cap_send.isOpened() or not out_send.isOpened():
        rospy.logerr(f"WARNING: unable to open video source for /dev/video{device}")
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
    global CAMERA_TYPE_INFO_BY_NAME
    rospy.init_node("cameras")

    raw_camera_type_info_by_name = rospy.get_param("cameras/camera_type_info")
    for name in raw_camera_type_info_by_name:
        info = raw_camera_type_info_by_name[name]
        quality_options = [
            CameraTypeInfo.QualityOption(q["width"], q["height"], q["fps"], q["bps"]) for q in info["quality_options"]
        ]
        CAMERA_TYPE_INFO_BY_NAME[name] = CameraTypeInfo(
            info["vendor_id"], info["vendor"], info["use_jpeg"], quality_options
        )

    stream_manager = StreamManager()
    rospy.Service("change_cameras", ChangeCameras, stream_manager.handle_req)
    rospy.Service("reset_cameras", ResetCameras, stream_manager.reset_streams)
    rospy.Timer(rospy.Duration(2), stream_manager.publish_available_cameras)

    rospy.spin()


if __name__ == "__main__":
    main()
