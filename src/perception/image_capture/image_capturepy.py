#!/usr/bin/env python3

import rospkg

import os

import sys
from pathlib import Path

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rospy

import cv2

import datetime

# ROS message types we need to use
from sensor_msgs.msg import Image

pano_tag = None

isPano = False
if len(sys.argv) > 1:
    isPano = True

    pano_tag = sys.argv[1]

rospack = rospkg.RosPack()
pkgPath = rospack.get_path("mrover")

if isPano:
    imagePath = pkgPath + f"/data/Images/pano/{pano_tag}/"
else:
    imagePath = pkgPath + f"/data/Images/scene/"

def on_press():
    msg = rospy.wait_for_message("/camera/left/image", Image, timeout=5)
    data = np.empty(msg.height * msg.width * 4, dtype=np.uint8)
    for x in range(msg.height * msg.width * 4):
        data[x] = msg.data[x]

    image = np.reshape(data, [msg.height, msg.width, 4])
    unique_id = "{date:%Y-%m-%d_%H:%M:%S}".format(date=datetime.datetime.now())

    fpath = Path(imagePath)
    if not fpath.exists():
        fpath.mkdir(exist_ok=True, parents=True)

    print(cv2.imwrite(imagePath + f"image_{unique_id}.jpg", image))


def on_release(key):
    pass


class image_capturepy:
    def __init__(self):
        while False:
            continue

def delete_files_in_directory(directory_path):
    try:
         if os.path.exists(imagePath):
            files = os.listdir(directory_path)
            for file in files:
                file_path = os.path.join(directory_path, file)
                if os.path.isfile(file_path):
                    os.remove(file_path)
                print("All files deleted successfully.")
    except OSError:
        print("Error occurred while deleting files.")

    

def main():
    # initialize the node
    rospy.init_node("image_capturepy")

    on_press()


if __name__ == "__main__":
    main()
