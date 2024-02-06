#!/usr/bin/env python3

from pynput.keyboard import Key, Controller

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rospy

import cv2

import datetime


# ROS message types we need to use
from sensor_msgs.msg import Image

import threading


class KeyboardThread(threading.Thread):

    def __init__(self, input_cbk=None, name="keyboard-input-thread"):
        self.input_cbk = input_cbk
        super(KeyboardThread, self).__init__(name=name)
        self.start()
        self.run = True

    def run(self):
        while True:
            self.input_cbk(input())  # waits to get input + Return
            if not self.run:
                break

    def stop(self):
        self.run = False


showcounter = 0  # something to demonstrate the change


def my_callback(inp):
    # evaluate the keyboard input
    image_capturepy.showcounter += 1


# start the Keyboard thread


class image_capturepy:
    showcounter = 0

    def __init__(self):
        rospy.Subscriber("/camera/left/image", Image, self.img_callback)
        self.kthread = KeyboardThread(my_callback)
        self.keyboard = Controller()

    def __del__(self):
        self.kthread.stop()
        self.keyboard.press(Key.enter)
        self.keyboard.release(Key.enter)

    def img_callback(self, msg):
        data = np.empty(msg.height * msg.width * 4, dtype=np.uint8)
        for x in range(msg.height * msg.width * 4):
            data[x] = msg.data[x]

        image = np.reshape(data, [msg.height, msg.width, 4])
        print(image_capturepy.showcounter)
        if image_capturepy.showcounter != 0:
            unique_id = "{date:%Y-%m-%d_%H:%M:%S}".format(date=datetime.datetime.now())
            print(cv2.imwrite(f"//home//john//Desktop//Rover//Images//image_{unique_id}.jpg", image))
            image_capturepy.showcounter = 0


def main():
    # initialize the node
    rospy.init_node("image_capturepy")

    # create and start our localization system
    image_capturepy1 = image_capturepy()

    # let the callback functions run asynchronously in the background
    rospy.spin()


if __name__ == "__main__":
    main()
