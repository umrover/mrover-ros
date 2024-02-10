#!/usr/bin/env python3

from pynput.keyboard import Controller, Key


# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rospy

import cv2

import datetime


# ROS message types we need to use
from sensor_msgs.msg import Image

# start the Keyboard thread


class image_capturepy:

    def __init__(self):
        self.keyboard = Controller()
        while True:
            input()
            msg = rospy.wait_for_message("/camera/left/image", Image, timeout=5)
            data = np.empty(msg.height * msg.width * 4, dtype=np.uint8)
            for x in range(msg.height * msg.width * 4):
                data[x] = msg.data[x]

            image = np.reshape(data, [msg.height, msg.width, 4])
            unique_id = "{date:%Y-%m-%d_%H:%M:%S}".format(date=datetime.datetime.now())
            print(cv2.imwrite(f"//home//john//Desktop//Rover//Images//image_{unique_id}.jpg", image))

    def __del__(self):
        self.keyboard.press(Key.enter)
        self.keyboard.release(Key.enter)


def main():
    # initialize the node
    rospy.init_node("image_capturepy")

    # create and start our localization system
    image_capturepy1 = image_capturepy()

    # let the callback functions run asynchronously in the background
    rospy.spin()


if __name__ == "__main__":
    main()
