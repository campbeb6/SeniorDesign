import sys

sys.path.append('/usr/local/lib/python2.7/site-packages') 

import numpy as np
import cv2

import time
import picamera
import picamera.array

from imutils.video.pivideostream import PiVideoStream
from picamera.array import PiRGBArray
from picamera import PiCamera
import imutils
import serial
import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import spidev
import math

from mazeSolve import MazeSolver
ser = serial.Serial('/dev/rfcomm0',9600)

xCells = 5
yCells = 5
start = [4,3]
end = [0,0]

ms = MazeSolver(xCells,yCells)

ms.getMaze()

vs = PiVideoStream().start()
time.sleep(2.0)


ledFront = ['0']
ledBack = ['1']
ledOff = ['2']
ledStraight = ['3']
ledLeft = ['4']
ledRight = ['5']
ledStop = ['6']


carAngle = 0
angleP = 0


cv2.namedWindow("Off")

foundFront = True
foundBack = True

done = False


#function for finding XY coordinates of car
def findCar():
        foundFront = False
        foundBack = False
        
        bPixel = [250,250];
        fPixel = [250,250];
        while not foundFront and not foundBack: 
    
            foundFront = False
            foundBack = False
    
            #turn on top
            ser.write(ledFront)
            print("Sent the message: {}".format(ledFront))
            time.sleep(0.3)
    
            #grab frame
            frontF = vs.read()
            ser.write(ledOff)
            print("Sent the message: {}".format(ledOff))
            time.sleep(0.1)
    
            #turn on bottom
            ser.write(ledBack)
            print("Sent the message: {}".format(ledBack))
            time.sleep(0.3)
    
            #grab frame
            backF = vs.read()
    
            #turn off all LEDs
            ser.write(ledOff)
            print("Sent the message: {}".format(ledOff))
            time.sleep(0.3)
    
            #grab frame
            offF = vs.read()

###################### Finding Car ######################

            fDiff = cv2.subtract(frontF, offF)
            bDiff = cv2.subtract(backF, offF)

            fGray = cv2.cvtColor(fDiff,cv2.COLOR_BGR2GRAY)
            ret,thresh1 = cv2.threshold(fGray,127,255,cv2.THRESH_BINARY)
            fLoc = np.argwhere(thresh1 == 255)
            if len(fLoc) > 0:
                fPixel = (fLoc[len(fLoc)/2])
                print(fPixel)
                foundFront = True

            bGray = cv2.cvtColor(bDiff,cv2.COLOR_BGR2GRAY)
            ret,thresh2 = cv2.threshold(bGray,127,255,cv2.THRESH_BINARY)
            bLoc = np.argwhere(thresh2 == 255)
            if len(bLoc) > 0 and len(fLoc) > 0:
                bPixel = (bLoc[len(bLoc)/2])
                print(bPixel)
                foundBack = True
        
                centerX = (fPixel[1]+bPixel[1])/2
                print(centerX)
                centerY = (fPixel[0]+bPixel[0])/2
                print(centerY)

        return [fPixel, bPixel, offF, centerX, centerY]

#################MAIN CODE STARTS HERE#########################


pixels = findCar()

carX = pixels[3]
carY = pixels[4]

ms.solveMaze(carX,carY)

coordinates = ms.finalCoords

point1 = coordinates.pop(0)
pointX = point1[0]
pointY = point1[1]



while True:
    pixels = findCar()
    
    fPixel = pixels[0]
    bPixel = pixels[1]
    offF = pixels[2]
    centerX = pixels[3]
    centerY = pixels[4]

    if not(fPixel[0]==bPixel[0] and fPixel[1]==bPixel[1]): 
        
        
###################### Direction of the Car ######################
        xf = fPixel[1]
        yf = fPixel[0]
        xb = bPixel[1]
        yb = bPixel[0]

        xDiff = abs(xf-centerX);
        yDiff = abs(yf-centerY);
        axis = centerX + xDiff;
        lineDist = math.sqrt((xDiff**2)+(yDiff**2))
        axisDist = math.sqrt((axis-centerX)**2)

        if ((yf <= yb) & (xf >= xb)):
            carAngle = math.degrees(math.acos(axisDist/lineDist))
            print(carAngle)
        if ((yf <= yb) & (xf < xb)):
            carAngle = 180-(math.degrees(math.acos(axisDist/lineDist)))
            print(carAngle)
        if ((yf > yb) & (xf <= xb)):
            carAngle = 180+(math.degrees(math.acos(axisDist/lineDist)))
            print(carAngle)
        if ((yf > yb) & (xf > xb)):
            carAngle = 360-(math.degrees(math.acos(axisDist/lineDist)))
            print(carAngle)
        
###################### Direction of the Point ######################

        xDiffP = abs(pointX - centerX);
        yDiffP = abs(pointY - centerY);
        axisP = centerX+xDiffP;
        lineDistP = math.sqrt((xDiffP**2)+(yDiffP**2))
        axisDistP = math.sqrt((axisP-centerX)**2)
        
        if(lineDistP <= 35):
            if(len(coordinates) == 0):
                done = True
            else:
                nextPoint = coordinates.pop(0)
                pointX = nextPoint[0]
                pointY = nextPoint[1]

        xDiffP = abs(pointX - centerX);
        yDiffP = abs(pointY - centerY);
        axisP = centerX+xDiffP;
        lineDistP = math.sqrt((xDiffP**2)+(yDiffP**2))
        axisDistP = math.sqrt((axisP-centerX)**2)
        
        print(lineDistP)

        if ((pointY <= centerY) & (pointX >= centerX)):
            angleP = math.degrees(math.acos(axisDistP/lineDistP))
            print(angleP)
        if ((pointY <= centerY) & (pointX < centerX)):
            angleP = 180-(math.degrees(math.acos(axisDistP/lineDistP)))
            print(angleP)
        if ((pointY > centerY) & (pointX <= centerX)):
            angleP = 180+(math.degrees(math.acos(axisDistP/lineDistP)))
            print(angleP)
        if ((pointY > centerY) & (pointX > centerX)):
            angleP = 360-(math.degrees(math.acos(axisDistP/lineDistP)))
            print(angleP)

        angleDiff = carAngle - angleP
        print(angleDiff)
    
###################### Directing the Car Straight ######################
        if angleDiff > 180:
            angleDiff -= 360
        elif angleDiff < -180:
            angleDiff += 360
        
        straightTime = lineDistP/80 * 0.4
        #if(lineDistP < 45 ):
            #straightTime = 0.15
        
        turnTime = abs(angleDiff)/60 * 0.3
        if(abs(angleDiff) < 30):
            turnTime = 0.15


        
        #turn if angle is past threshold
        if(abs(angleDiff)<=18):
            ser.write(ledStraight)
            time.sleep(straightTime)
            print("going straight")
            ser.write(ledStop)
            time.sleep(0.1)

        elif(angleDiff >= 18):
            ser.write(ledRight)
            print("turning right")
            time.sleep(turnTime)
            ser.write(ledStop)
            #print("Sent the message: {}".format(ledStop))
            time.sleep(0.1)
        
        elif(angleDiff <= -18):
            ser.write(ledLeft)
            print("turning left")
            time.sleep(turnTime)
            ser.write(ledStop)
            time.sleep(0.1)
       
        try:
            while done:
                ser.write(ledRight)
                ser.write(ledFront)
                time.sleep(0.05)
                ser.write(ledBack)
                time.sleep(0.05)
                print("PARTY!")
        except KeyboardInterrupt:
            print("GOODBYE!")
            ser.write(ledStop)
            ser.write(ledOff)
            break
###################### Directing the Car to Turn ######################
             
    cv2.circle(offF,(centerX,centerY),3,(255,0,0),2,-1)
    cv2.circle(offF,(pointX,pointY),3,(0,255,0),2,-1)
    cv2.imshow("Off", offF)
   # cv2.imshow("back", backF)
   # cv2.imshow("front binary", thresh1)
   # cv2.imshow("back binary", thresh2)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break
        
           
cv2.destroyAllWindows()
vs.stop()

   
