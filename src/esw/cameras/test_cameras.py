#!/usr/bin/env python3

import jetson.utils

device = 0
args = ["--headless", "--bitrate=300000", "--width=256", "--height=144"]


def main():
    video_source = jetson.utils.videoSource(f"/dev/video{device}", argv=args)
    video_output = jetson.utils.videoOutput(f"rtp://10.0.0.7:5000", argv=args)
    while True:
        image = video_source.Capture()
        video_output.Render(image)


if __name__ == "__main__":
    main()
