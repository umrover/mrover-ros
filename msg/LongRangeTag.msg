# Message published by the LongRangeTagDetector nodelet. 
# -0.5 <= xOffset, yOffset <= 0.5
# Normed to image size, and have signs reflecting typical cartesian coordinates
# Where (0, 0) is at the image center
uint8 id
float32 xOffset
float32 yOffset