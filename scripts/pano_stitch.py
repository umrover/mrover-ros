import cv2
import sys

from pathlib import Path 

import os


pano_id = 0

if len(sys.argv) > 1:
    pano_id = sys.argv[1]
else:
    raise Exception("No arg provided")

PANO_PATH = Path(f'data/Images/pano/{pano_id}/')

image_paths = os.listdir(PANO_PATH)

images = [cv2.imread(str(PANO_PATH) + "/" + image_path) for image_path in image_paths]

stitcher = cv2.Stitcher.create()

status, pano = stitcher.stitch(images)

cv2.imwrite(str(PANO_PATH) + "/" + "panorama.jpg", pano)       