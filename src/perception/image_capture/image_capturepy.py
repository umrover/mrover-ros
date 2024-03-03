#!/usr/bin/env python3

from pynput import keyboard

import rospkg

import os

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rospy

import cv2

import datetime


# ROS message types we need to use
from sensor_msgs.msg import Image


def on_press(key):
    if key == keyboard.Key.enter:
        msg = rospy.wait_for_message("/camera/left/image", Image, timeout=5)
        data = np.empty(msg.height * msg.width * 4, dtype=np.uint8)
        for x in range(msg.height * msg.width * 4):
            data[x] = msg.data[x]

        image = np.reshape(data, [msg.height, msg.width, 4])
        unique_id = "{date:%Y-%m-%d_%H:%M:%S}".format(date=datetime.datetime.now())
        rospack = rospkg.RosPack()
        pkgPath = rospack.get_path("mrover")
        imagePath = pkgPath + f"/data/Images"
        if not os.path.exists(imagePath):
            os.mkdir(imagePath)

        print(cv2.imwrite(imagePath + f"image_{unique_id}.jpg", image))


def on_release(key):
    pass


class image_capturepy:
    def __init__(self):
        while False:
            continue


def main():
    # initialize the node
    rospy.init_node("image_capturepy")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    # let the callback functions run asynchronously in the background
    rospy.spin()


if __name__ == "__main__":
    main()
