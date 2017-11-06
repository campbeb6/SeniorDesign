import sys

sys.path.append('/usr/local/lib/python2.7/site-packages')

#import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import copy
import numpy as np

#initialize the camera and grab a reference to the camera raw capture
camera = PiCamera()
camera.resolution = (320,240)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(320,240))

#allow the camera to warmup
time.sleep(0.1)

lower_green = np.array([30,50,50])
upper_green = np.array([50,255,255])

lower_orange = np.array([5,50,50])
upper_orange = np.array([25,255,255])

cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
cv2.namedWindow("line segments", cv2. WINDOW_NORMAL)
cv2.namedWindow("mask1", cv2.WINDOW_NORMAL)
cv2.namedWindow("mask2", cv2.WINDOW_NORMAL)
cv2.resizeWindow("mask1", 600, 450)
cv2.resizeWindow("mask2", 600, 450)
cv2.resizeWindow("Frame", 600, 450)
cv2.resizeWindow("line segments", 600, 450)


lsd = cv2.createLineSegmentDetector(0);

#capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
   #grab the raw NumPy array representing the image, then initialize the timestamp
   #and occupied/unoccupied text
    image = frame.array

    image = cv2.flip(image,0)
    image2 = image.copy()

    #convert to hsv for color line detection
    hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    blur = cv2.medianBlur(hsv,5)

    #mask based on line color
    mask1 = cv2.inRange(blur, lower_green, upper_green)
    mask2 = cv2.inRange(blur, lower_orange, upper_orange)

    _,contours, hierarchy = cv2.findContours(mask2, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    #finding contour with maximum area
    max_area = 0
    for cnt in contours:
	area = cv2.contourArea(cnt)
	if area > max_area:
	    max_area = area
	    best_cnt = cnt

    #detect line segments in mask
    lines = lsd.detect(mask1)[0]

    #draw line segments on original image
    drawn = lsd.drawSegments(image, lines)


    cv2.imshow("Frame", image2)
    cv2.imshow("line segments", drawn)
    cv2.imshow("mask1", mask1)
    cv2.imshow("mask2", mask2)
    key = cv2.waitKey(1) & 0xFF

    rawCapture.truncate(0)

    if key == ord("q"):
        break


