import sys

sys.path.append('/usr/local/lib/python2.7/site-packages')

from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import serial
import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import spidev
import math

GPIO.setmode(GPIO.BCM)

pipes = [[0xE8, 0xE8, 0xF0, 0xF0, 0xE1], [0xF0, 0xF0, 0xF0, 0xF0, 0xE1]]
radio = NRF24 (GPIO, spidev.SpiDev())
radio.begin(0,17)
radio.setPayloadSize(1)
radio.setChannel(0x75)
radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MIN)
radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()
radio.openWritingPipe(pipes[0])
radio.openReadingPipe(1, pipes[1])

def findCar():
    # Initialize the camera and grab a reference to the camera raw capture
    camera = PiCamera()
    rawCaptureOff = PiRGBArray(camera)
    rawCapture = PiRGBArray(camera)
    rawCapture2 = PiRGBArray(camera)
    time.sleep(0.1)

    ledFront = ['e']
    ledBack = ['f']
    ledOff = ['g']

    camera.capture(rawCaptureOff, format="bgr")
    off = rawCaptureOff.array
    cv2.imwrite("/home/pi/Desktop/PYSCRIPTS/off.jpg", off)

    radio.write(ledFront)
    print("Sent the message: {}".format(ledFront))
    time.sleep(1)
    camera.capture(rawCapture, format="bgr")
    fOn = rawCapture.array
    cv2.imwrite("/home/pi/Desktop/PYSCRIPTS/front.jpg", fOn)

    radio.write(ledBack)
    print("Sent the message: {}".format(ledBack))
    time.sleep(1)
    camera.capture(rawCapture2, format="bgr")
    bOn = rawCapture2.array
    cv2.imwrite("/home/pi/Desktop/PYSCRIPTS/back.jpg", bOn)

    radio.write(ledOff)
    print("Sent the message: {}".format(ledOff))
    time.sleep(1)

###################### Finds location of front/back pixel (y, x) ######################

    fDiff = cv2.subtract(fOn, off)
    bDiff = cv2.subtract(bOn, off)

    fGray = cv2.cvtColor(fDiff,cv2.COLOR_BGR2GRAY)
    ret,thresh1 = cv2.threshold(fGray,127,255,cv2.THRESH_BINARY)
    cv2.imwrite("fBinary.jpg", thresh1)
    fLoc = np.argwhere(thresh1 == 255)
    fPixel = (fLoc[len(fLoc)/2])
    print(fPixel)

    bGray = cv2.cvtColor(bDiff,cv2.COLOR_BGR2GRAY)
    ret,thresh2 = cv2.threshold(bGray,127,255,cv2.THRESH_BINARY)
    cv2.imwrite("bBinary.jpg", thresh2)
    bLoc = np.argwhere(thresh2 == 255)
    bPixel = (bLoc[len(bLoc)/2])
    print(bPixel)
    

###################### Direction ######################

    q1 = False
    q2 = False
    q3 = False
    q4 = False

    xf = fPixel[1]
    yf = fPixel[0]
    xb = bPixel[1]
    yb = bPixel[0]

    xDiff = abs(xf-xb);
    yDiff = abs(yf-yb);
    axis = xb+xDiff;
    lineDist = math.sqrt((xDiff**2)+(yDiff**2))
    axisDist = math.sqrt((axis-xb)**2)

    if (yf <= yb) & (xf >= xb):
        q1 = True
    if (yf <= yb) & (xf < xb):
        q2 = True
    if (yf > yb) & (xf <= xb):
        q3 = True
    if (yf > yb) & (xf > xb):
        q4 = True

    if q1 == True:
        angle = math.degrees(math.acos(axisDist/lineDist))
        print(angle)
    if q2 == True:
        angle = 180-(math.degrees(math.acos(axisDist/lineDist)))
        print(angle)
    if q3 == True:
        angle = 180+(math.degrees(math.acos(axisDist/lineDist)))
        print(angle)
    if q4 == True:
        angle = 360-(math.degrees(math.acos(axisDist/lineDist)))
        print(angle)


###################### Car Center ######################

    centerX = (xf+xb)/2
    centerY = (yf+yb)/2
    print(centerX)
    print(centerY)
    return
