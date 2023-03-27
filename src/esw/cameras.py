#!/usr/bin/env python3

from mrover.msg import CameraCmd
import cv2
from multiprocessing import Process

import rospy
from mrover.srv import (
    ChangeCameras,
    ChangeCamerasRequest,
    ChangeCamerasResponse,
)


class StreamManager:

    def __init__(self):
        self.stream_process_list = \
            [[0 for _ in range(rospy.get_param("cameras/max_video_device_id_number"))] for __ in range(2)]
        self.ips = [rospy.get_param("cameras/ips/primary"), rospy.get_param("cameras/ips/secondary")]
        self.cap_args = [
            # [bps, width, height, fps]
            list(rospy.get_param("cameras/arguments/worst_res")),
            list(rospy.get_param("cameras/arguments/low_res")),
            list(rospy.get_param("cameras/arguments/medium_res")),
            list(rospy.get_param("cameras/arguments/high_res")),
            list(rospy.get_param("cameras/arguments/best_res")),
        ]
        self.skip_every_other_device = rospy.get_param("cameras/skip_every_other_device")

    def handle_req(self, req):
        cmds = req.camera_cmds
        laptop_idx = 0 if req.primary else 1
        device_id = cmds.device
        print(device_id)
        cap = cmds.resolution

        if cap:
            print(self.stream_process_list[laptop_idx][device_id])
            if self.stream_process_list[laptop_idx][device_id]:
                print("Killing existing")
                self.stream_process_list[laptop_idx][device_id].kill()
                self.stream_process_list[laptop_idx][device_id] = 0
                self.stream_process_list[laptop_idx][device_id].join()
                print("Killed existing")
            self.stream_process_list[laptop_idx][device_id] = Process(
                target=send,
                args=(
                    device_id * 2 if self.skip_every_other_device else device_id,
                    self.ips[laptop_idx],
                    5000 + device_id,
                    self.cap_args[cap - 1][0],
                    self.cap_args[cap - 1][1],
                    self.cap_args[cap - 1][2],
                    self.cap_args[cap - 1][3],
                    True,
                ),
            )
            self.stream_process_list[laptop_idx][device_id].start()
        else:
            print("\nClosing /dev/video" + str(device_id) + " stream")
            self.stream_process_list[laptop_idx][device_id].kill()
            self.stream_process_list[laptop_idx][device_id] = 0


def send(device=0, host="10.0.0.7", port=5000, bitrate=4000000, width=1280, height=720, fps=30, is_colored=False):
    # TODO: Look into bitrate "automatic" calculation

    # Construct video capture pipeline string
    cap_str = (
        "v4l2src device=/dev/video"
        + str(device)
        + " do-timestamp=true io-mode=2 ! \
    image/jpeg, width="
        + str(width)
        + ", height="
        + str(height)
        + ", framerate="
        + str(fps)
        + "/1 ! \
    jpegdec ! \
    videorate ! \
    video/x-raw,\
    framerate="
        + str(fps)
        + "/1 ! \
    nvvidconv ! "
    )
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

    # openCV stream transmit pipeline with RTP sink
    fourcc = cv2.VideoWriter_fourcc("H", "2", "6", "4")
    out_send = cv2.VideoWriter(txstr, cv2.CAP_GSTREAMER, fourcc, 60, (width, height), is_colored)

    print(
        "\nTransmitting /dev/video"
        + str(device)
        + " to "
        + host
        + ":"
        + str(port)
        + " with "
        + str(bitrate / 1e6)
        + " Mbps target, "
        + str(fps)
        + " fps target, ("
        + str(width)
        + ","
        + str(height)
        + ") resolution\n"
    )

    if not cap_send.isOpened() or not out_send.isOpened():
        print("\nWARNING: unable to open video source for /dev/video" + str(device) + "\n")
        exit(0)

    # Transmit loop
    while True:

        ret, frame = cap_send.read()
        if not ret:
            print("empty frame")
            break
        out_send.write(frame)

    print("stream machine broke")
    cap_send.release()
    out_send.release()


if __name__ == "__main__":
    rospy.init_node("cameras")
    stream_manager = StreamManager()
    rospy.Service("change_cameras", ChangeCameras, stream_manager.handle_req)
    rospy.spin()
