# Uses openCV with gstreamer support to capture video from v4l2 device and transmit over RTP
# Intended to be used with RXstream.py on a device with ethernet NIC ip 10.0.0.7 and over a properly configured network

import cv2
from multiprocessing import Process
from pynput import keyboard

lk = ""


def stream_manager(stream_process_list, id, cap):
    cap_args = [
        # [bps, width, height, fps]
        [173000, 320, 240, 15],  # bottom
        [691000, 640, 480, 15],  # low
        [2000000, 960, 720, 15],  # medium
        [3000000, 1280, 720, 15],  # high
        [4200000, 1280, 720, 30],  # full
    ]
    if cap:
        if stream_process_list[id]:
            stream_process_list[id].kill()
        stream_process_list[id] = Process(
            target=send,
            args=(
                id,
                "10.0.0.7",
                5000 + id,
                cap_args[cap - 1][0],
                cap_args[cap - 1][1],
                cap_args[cap - 1][2],
                cap_args[cap - 1][3],
                True,
            ),
        )
        stream_process_list[id].start()
    else:
        print("\nClosing /dev/video" + str(id) + " stream")
        stream_process_list[id].kill()


def send(device=0, host="10.0.0.7", port=5001, bitrate=4000000, width=1280, height=720, fps=30, isColored=False):
    # TODO: Look into bitrate "automatic" calculation

    # Construct video capture pipeline string
    capstr = (
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
    if isColored:
        capstr += " video/x-raw, format=BGRx ! "
    capstr += "videoconvert ! "
    if isColored:
        capstr += " video/x-raw, format=BGR ! "
    capstr += "appsink"

    # openCV video capture from v4l2 device
    cap_send = cv2.VideoCapture(capstr, cv2.CAP_GSTREAMER)

    # Construct stream transmit pipeline string
    txstr = "appsrc ! "
    if isColored:
        txstr += " video/x-raw, format=BGR ! "
    txstr += "videoconvert ! "
    if isColored:
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
    out_send = cv2.VideoWriter(txstr, cv2.CAP_GSTREAMER, fourcc, 60, (width, height), isColored)

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


def on_press(key):  # keyboard keypress event handler
    global lk
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    numlist = [str(n) for n in list(range(10))]
    if k in numlist:
        print("Controlling /dev/video" + k)
    if lk in numlist:
        if k in ["q"]:
            stream_manager(s, int(lk), 0)
        if k in ["b"]:
            stream_manager(s, int(lk), 1)
        if k in ["l"]:
            stream_manager(s, int(lk), 2)
        if k in ["m"]:
            stream_manager(s, int(lk), 3)
        if k in ["h"]:
            stream_manager(s, int(lk), 4)
        if k in ["f"]:
            stream_manager(s, int(lk), 5)
    lk = k


if __name__ == "__main__":
    s = [0] * 10
    s[0] = Process(target=send, args=(0, "10.0.0.7", 5000, 4200000, 1280, 720, 30, True))
    s[2] = Process(target=send, args=(2, "10.0.0.7", 5002, 4200000, 1280, 720, 30, True))
    cthread = keyboard.Listener(on_press=on_press, args=(s))
    s[0].start()
    s[2].start()
    cthread.start()
    print(
        "[KEYBOARD CONTROL INSTRUCTIONS]\n\
    Press a number to control dev/video[number]\n\
    Press B for worst capture setting\n\
    Press L for low capture settings\n\
    Press M for medium capture settings\n\
    Press H for high capture settings\n\
    Press F for best capture setting \n\
    Press Q to close selected stream\n\n"
    )
    for i in range(len(s)):
        if s[i]:
            s[i].join()

    cthread.join()
    cv2.destroyAllWindows()
