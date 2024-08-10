import cv2

from pathlib import Path 

import os

PANO_PATH = Path('data/Images/pano')

image_paths = os.listdir(PANO_PATH)

images = [cv2.imread('data/Images/pano/' + image_path) for image_path in image_paths]

stitcher = cv2.Stitcher.create()

status, pano = stitcher.stitch(images)

cv2.imwrite("data/Images/pano/panorama.jpg", pano)       